#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/server_builder.h>
#include <proto/VehicleTrajectoryService.grpc.pb.h>
#include <proto/VehicleTrajectoryService.pb.h>
#include <trajopt/OptimalTrajectoryGenerator.h>
#include <trajopt/drivetrain/SwerveDrivetrain.h>
#include <trajopt/path/SwervePathBuilder.h>

namespace vts = org::littletonrobotics::vehicletrajectoryservice;

trajopt::SwerveDrivetrain create_drivetrain(const vts::VehicleModel &model) {
    trajopt::SwerveDrivetrain drivetrain{
            .mass = model.mass(),
            .moi = model.moi(),
            .modules = {{.x = model.vehicle_length() / 2.0,
                                .y = model.vehicle_width() / 2.0,
                                .wheelRadius = model.wheel_radius(),
                                .wheelMaxAngularVelocity = model.max_wheel_omega(),
                                .wheelMaxTorque = model.max_wheel_torque()},
                        {.x = model.vehicle_length() / 2.0,
                                .y = -model.vehicle_width() / 2.0,
                                .wheelRadius = model.wheel_radius(),
                                .wheelMaxAngularVelocity = model.max_wheel_omega(),
                                .wheelMaxTorque = model.max_wheel_torque()},
                        {.x = -model.vehicle_length() / 2.0,
                                .y = model.vehicle_width() / 2.0,
                                .wheelRadius = model.wheel_radius(),
                                .wheelMaxAngularVelocity = model.max_wheel_omega(),
                                .wheelMaxTorque = model.max_wheel_torque()},
                        {.x = -model.vehicle_length() / 2.0,
                                .y = -model.vehicle_width() / 2.0,
                                .wheelRadius = model.wheel_radius(),
                                .wheelMaxAngularVelocity = model.max_wheel_omega(),
                                .wheelMaxTorque = model.max_wheel_torque()}}};

    return drivetrain;
}

int guess_control_interval_count(const vts::Waypoint &waypoint, const vts::Waypoint &prev_waypoint,
                                 const vts::VehicleModel &model) {
    double dx = waypoint.x() - prev_waypoint.x();
    double dy = waypoint.y() - prev_waypoint.y();
    double dtheta = waypoint.has_heading_constraint() && prev_waypoint.has_heading_constraint() ?
                    waypoint.heading_constraint() - prev_waypoint.heading_constraint() : 0;
    static const double heading_weight = 0.5;
    double distance = hypot(dx, dy);
    double max_force = model.max_wheel_torque() / model.wheel_radius();
    double max_accel = (max_force * 4) / model.mass();
    double max_vel = model.max_wheel_omega() * model.wheel_radius();
    double distance_at_cruise = distance - (max_vel * max_vel) / max_accel;
    double total_time = distance_at_cruise < 0 ?
                        2 * (sqrt(distance * max_accel) / max_accel) :
                        distance / max_vel + max_vel / max_accel;
    total_time += heading_weight * abs(dtheta);
    return ceil(total_time / 0.1);
}

void convert_sample(vts::TimestampedVehicleState *state_out, const trajopt::HolonomicTrajectorySample &sample_in) {
    state_out->set_time(sample_in.timestamp);
    vts::VehicleState *vehicle_state = state_out->mutable_state();
    vehicle_state->set_x(sample_in.x);
    vehicle_state->set_y(sample_in.y);
    vehicle_state->set_theta(sample_in.heading);
    vehicle_state->set_vx(sample_in.velocityX);
    vehicle_state->set_vy(sample_in.velocityY);
    vehicle_state->set_omega(sample_in.angularVelocity);
}

void convert_trajectory(vts::Trajectory *trajectory_out, const trajopt::HolonomicTrajectory &trajectory_in) {
    for (const trajopt::HolonomicTrajectorySample &sample: trajectory_in.samples) {
        convert_sample(trajectory_out->add_states(), sample);
    }
}

std::string hash_request(const vts::PathRequest &request) {
    // Function is somewhat messy but this is the simplest way I could think of to hash the request with
    // fixed precision while accounting for optional fields

    std::stringstream stream;
    stream << std::fixed << std::setprecision(6);
    stream << request.model().mass();
    stream << request.model().moi();
    stream << request.model().vehicle_length();
    stream << request.model().vehicle_width();
    stream << request.model().wheel_radius();
    stream << request.model().max_wheel_omega();
    stream << request.model().max_wheel_torque();

    for (const vts::PathSegment &segment: request.segments()) {
        for (const vts::Waypoint &waypoint: segment.waypoints()) {
            stream << waypoint.x();
            stream << waypoint.y();

            if (waypoint.has_heading_constraint()) {
                stream << waypoint.heading_constraint();
            }

            if (waypoint.has_samples()) {
                stream << waypoint.samples();
            }

            switch (waypoint.velocity_constraint_case()) {
                case vts::Waypoint::kZeroVelocity:
                    stream << "0";
                    break;
                case vts::Waypoint::kVehicleVelocity:
                    stream << waypoint.vehicle_velocity().vx();
                    stream << waypoint.vehicle_velocity().vy();
                    stream << waypoint.vehicle_velocity().omega();
                    break;
                case org::littletonrobotics::vehicletrajectoryservice::Waypoint::VELOCITY_CONSTRAINT_NOT_SET:
                    break;
            }
        }

        if (segment.has_max_velocity()) {
            stream << segment.max_velocity();
        }

        if (segment.has_max_omega()) {
            stream << segment.max_omega();
        }

        if (segment.straight_line()) {
            stream << "1";
        } else {
            stream << "0";
        }
    }

    return std::to_string(std::hash<std::string>{}(stream.str()));
}

class VehicleTrajectoryService final
        : public vts::VehicleTrajectoryService::Service {
public:
    ::grpc::Status GenerateTrajectory(::grpc::ServerContext *context,
                                      const vts::PathRequest *request,
                                      vts::TrajectoryResponse *response) override {
        trajopt::SwervePathBuilder builder;
        builder.SetDrivetrain(create_drivetrain(request->model()));
        std::vector<size_t> control_intervals;

        int segment_start_offset = 0;
        for (int segment_idx = 0; segment_idx < request->segments_size(); segment_idx++) {
            fmt::print("Starting segment {}\n", segment_idx);
            // Add segment waypoints
            const vts::PathSegment &segment = request->segments(segment_idx);
            const int last_waypoint_idx = segment.waypoints_size() - 1;
            const vts::Waypoint *prev_waypoint;

            for (int waypoint_idx = 0; waypoint_idx < segment.waypoints_size(); waypoint_idx++) {
                const vts::Waypoint &waypoint = segment.waypoints(segment_start_offset + waypoint_idx);

                if (waypoint.has_heading_constraint()) {
                    fmt::print("Adding pose waypoint {} ({}, {}, {})\n", segment_start_offset + waypoint_idx,
                               waypoint.x(), waypoint.y(), waypoint.heading_constraint());
                    builder.PoseWpt(segment_start_offset + waypoint_idx, waypoint.x(), waypoint.y(),
                                    waypoint.heading_constraint());
                } else {
                    fmt::print("Adding translation waypoint {} ({}, {})\n", segment_start_offset + waypoint_idx,
                               waypoint.x(), waypoint.y());
                    builder.TranslationWpt(segment_start_offset + waypoint_idx, waypoint.x(), waypoint.y());
                }

                switch (waypoint.velocity_constraint_case()) {
                    case vts::Waypoint::VELOCITY_CONSTRAINT_NOT_SET:
                        // If this is the first or last waypoint, add an implicit stop if no other constraint is added
                        if (waypoint_idx == 0 || waypoint_idx == last_waypoint_idx) {
                            fmt::print("Adding implicit zero velocity constraint to waypoint {}\n",
                                       segment_start_offset + waypoint_idx);
                            builder.WptZeroVelocity(segment_start_offset + waypoint_idx);
                        }
                        break;
                    case vts::Waypoint::kZeroVelocity:
                        fmt::print("Adding zero velocity constraint to waypoint {}\n",
                                   segment_start_offset + waypoint_idx);
                        builder.WptZeroVelocity(segment_start_offset + waypoint_idx);
                        break;
                    case vts::Waypoint::kVehicleVelocity:
                        fmt::print("Adding vehicle velocity constraint ({}, {}, {}) to waypoint {}\n",
                                   waypoint.vehicle_velocity().vx(), waypoint.vehicle_velocity().vy(),
                                   waypoint.vehicle_velocity().omega(), segment_start_offset + waypoint_idx);
                        builder.WptConstraint(segment_start_offset + waypoint_idx,
                                              trajopt::HolonomicVelocityConstraint{
                                                      trajopt::RectangularSet2d{
                                                              {waypoint.vehicle_velocity().vx(),
                                                                      waypoint.vehicle_velocity().vx()},
                                                              {waypoint.vehicle_velocity().vy(),
                                                                      waypoint.vehicle_velocity().vy()},
                                                      },
                                                      trajopt::CoordinateSystem::kField
                                              });
                        builder.WptConstraint(segment_start_offset + waypoint_idx,
                                              trajopt::AngularVelocityConstraint{
                                                      trajopt::IntervalSet1d{
                                                              waypoint.vehicle_velocity().omega(),
                                                              waypoint.vehicle_velocity().omega()
                                                      }
                                              });
                        break;
                }

                if (segment_idx > 0 || waypoint_idx > 0) {
                    unsigned int sample_count = waypoint.has_samples() ? waypoint.samples()
                                                                       : guess_control_interval_count(waypoint,
                                                                                                      *prev_waypoint,
                                                                                                      request->model());
                    fmt::print("Adding sample count {} between waypoints {}-{}\n", sample_count, waypoint_idx - 1,
                               waypoint_idx);
                    control_intervals.push_back(sample_count);
                }

                prev_waypoint = &waypoint;
            }

            // Segment constraints
            if (segment.has_max_velocity()) {
                fmt::print("Adding max velocity {} to segment {} (waypoints {}-{})\n", segment.max_velocity(),
                           segment_idx, segment_start_offset, segment_start_offset + last_waypoint_idx);
                builder.SgmtVelocityMagnitude(segment_start_offset, segment_start_offset + last_waypoint_idx,
                                              segment.max_velocity());
            }

            if (segment.has_max_omega()) {
                fmt::print("Adding max omega {} to segment {} (waypoints {}-{})\n", segment.max_omega(),
                           segment_idx, segment_start_offset, segment_start_offset + last_waypoint_idx);
                builder.SgmtConstraint(segment_start_offset, segment_start_offset + last_waypoint_idx,
                                       trajopt::AngularVelocityConstraint{segment.max_omega()});
            }

            if (segment.straight_line()) {
                double x1 = segment.waypoints(segment_start_offset).x();
                double x2 = segment.waypoints(segment_start_offset + last_waypoint_idx).x();
                double y1 = segment.waypoints(segment_start_offset).y();
                double y2 = segment.waypoints(segment_start_offset + last_waypoint_idx).y();
                double angle = atan2(y2 - y1, x2 - x1);
                fmt::print("Adding straight line constraint with angle {} to segment {} (waypoints {}-{})\n", angle,
                           segment_idx, segment_start_offset, segment_start_offset + last_waypoint_idx);
                builder.SgmtVelocityDirection(segment_start_offset, segment_start_offset + last_waypoint_idx,
                                              angle);
            }

            segment_start_offset = last_waypoint_idx;
        }

        builder.ControlIntervalCounts(std::move(control_intervals));

        try {
            fmt::print("Generating trajectory\n");
            trajopt::HolonomicTrajectory trajectory{trajopt::OptimalTrajectoryGenerator::Generate(builder)};
            fmt::print("Generation finished\n");
            convert_trajectory(response->mutable_trajectory(), trajectory);
            response->mutable_trajectory()->set_hash_code(hash_request(*request));
        } catch (std::exception &e) {
            fmt::print("Generation failed: {}\n", std::string(e.what()));
            response->mutable_error()->set_reason(std::string(e.what()));
        }

        return grpc::Status::OK;
    }
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
    VehicleTrajectoryService service;

    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ::grpc::ServerBuilder builder;
    builder.AddListeningPort("0.0.0.0:56328", grpc::InsecureServerCredentials());

    builder.RegisterService(&service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    server->Wait();

    return 0;
}