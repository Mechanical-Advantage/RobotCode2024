#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/server_builder.h>
#include <proto/VehicleTrajectoryService.grpc.pb.h>
#include <proto/VehicleTrajectoryService.pb.h>
#include <trajopt/OptimalTrajectoryGenerator.h>
#include <trajopt/drivetrain/SwerveDrivetrain.h>
#include <trajopt/path/SwervePathBuilder.h>
#include <trajopt/trajectory/HolonomicTrajectorySample.h>

#include <fmt/format.h>
#include <numbers>
#include <csignal>

namespace vts = org::littletonrobotics::vehicletrajectoryservice;

// Scales the Choreo algorithm's estimate of initial guess points, since we've found it to sometimes underestimate.
// We take the tradeoff of increased computation time since we cache paths anyway.
static const double CONTROL_INTERVAL_GUESS_SCALAR = 1.1;

static const int MINIMUM_CONTROL_INTERVAL_COUNT = 0;

static std::unique_ptr<grpc::Server> server;

static void signal_handler(int signal) {
    server->Shutdown();
}

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
    return ceil(fmax(MINIMUM_CONTROL_INTERVAL_COUNT, total_time / 0.1 * CONTROL_INTERVAL_GUESS_SCALAR));
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

void convert_solution(vts::Trajectory *trajectory_out, const trajopt::SwerveSolution &solution_in) {
    double ts = 0.0;
    for (size_t samp = 0; samp < solution_in.x.size(); samp++) {
        if (samp != 0) {
            ts += solution_in.dt[samp - 1];
        }

        vts::TimestampedVehicleState *timestamped_state = trajectory_out->add_states();
        timestamped_state->set_time(ts);

        vts::VehicleState *vehicle_state = timestamped_state->mutable_state();
        vehicle_state->set_x(solution_in.x.at(samp));
        vehicle_state->set_y(solution_in.y.at(samp));
        vehicle_state->set_theta(solution_in.theta.at(samp));
        vehicle_state->set_vx(solution_in.vx.at(samp));
        vehicle_state->set_vy(solution_in.vy.at(samp));
        vehicle_state->set_omega(solution_in.omega.at(samp));

        for (int module_idx = 0; module_idx < solution_in.moduleFX.at(samp).size(); module_idx++) {
            vts::ModuleForce *module_force = vehicle_state->add_module_forces();
            module_force->set_fx(solution_in.moduleFX.at(samp).at(module_idx));
            module_force->set_fy(solution_in.moduleFY.at(samp).at(module_idx));
        }
    }
}

double angle_modulus(double value) {
    double minInput = -std::numbers::pi;
    double maxInput = std::numbers::pi;
    double modulus = maxInput - minInput;

    int numMax = (value - minInput) / modulus;
    value -= numMax * modulus;
    int numMin = (value - maxInput) / modulus;
    value -= numMin * modulus;
    return value;
}

std::vector<vts::Waypoint> add_waypoints(trajopt::SwervePathBuilder &builder, const vts::PathRequest &request) {
    int full_rots = 0;
    double prev_heading = 0;
    bool found_prev_heading = false;
    int idx = 0;
    std::vector<vts::Waypoint> all_wpts;

    for (const vts::PathSegment &segment: request.segments()) {
        for (const vts::Waypoint &waypoint: segment.waypoints()) {
            if (waypoint.has_heading_constraint()) {
                if (!found_prev_heading) {
                    prev_heading = waypoint.heading_constraint();
                    found_prev_heading = true;
                }
                double prev_heading_mod = angle_modulus(prev_heading);
                double heading_mod = angle_modulus(waypoint.heading_constraint());
                if (prev_heading_mod < 0 && heading_mod > prev_heading_mod + std::numbers::pi) {
                    full_rots--;
                } else if (prev_heading_mod > 0 && heading_mod < prev_heading_mod - std::numbers::pi) {
                    full_rots++;
                }
                double heading = full_rots * 2 * std::numbers::pi + heading_mod;
                prev_heading = waypoint.heading_constraint();

                fmt::print("Adding pose waypoint {} ({}, {}, {})\n", idx, waypoint.x(), waypoint.y(), heading);
                builder.PoseWpt(idx, waypoint.x(), waypoint.y(), heading);
            } else {
                fmt::print("Adding translation waypoint {} ({}, {})\n", idx, waypoint.x(), waypoint.y());
                builder.TranslationWpt(idx, waypoint.x(), waypoint.y());
            }

            all_wpts.push_back(waypoint);
            idx++;
        }
    }

    return std::move(all_wpts);
}

void apply_waypoint_constraints(trajopt::SwervePathBuilder &builder, const std::vector<vts::Waypoint> &waypoints,
                                const vts::VehicleModel &model) {
    std::vector<size_t> control_intervals;

    for (int i = 0; i < waypoints.size(); i++) {
        const vts::Waypoint &waypoint = waypoints.at(i);

        switch (waypoint.velocity_constraint_case()) {
            case vts::Waypoint::VELOCITY_CONSTRAINT_NOT_SET:
                // If this is the first or last waypoint, add an implicit stop if no other constraint is added
                if (i == 0 || i == waypoints.size() - 1) {
                    fmt::print("Adding implicit zero velocity constraint to waypoint {}\n", i);
                    builder.WptZeroVelocity(i);
                    builder.WptZeroAngularVelocity(i);
                }
                break;
            case vts::Waypoint::kZeroVelocity:
                fmt::print("Adding zero velocity constraint to waypoint {}\n", i);
                builder.WptZeroVelocity(i);
                builder.WptZeroAngularVelocity(i);
                break;
            case vts::Waypoint::kVehicleVelocity:
                fmt::print("Adding vehicle velocity constraint ({}, {}, {}) to waypoint {}\n",
                           waypoint.vehicle_velocity().vx(), waypoint.vehicle_velocity().vy(),
                           waypoint.vehicle_velocity().omega(), i);
                builder.WptConstraint(i,
                                      trajopt::HolonomicVelocityConstraint{
                                              trajopt::RectangularSet2d{
                                                      {waypoint.vehicle_velocity().vx(),
                                                              waypoint.vehicle_velocity().vx()},
                                                      {waypoint.vehicle_velocity().vy(),
                                                              waypoint.vehicle_velocity().vy()},
                                              },
                                              trajopt::CoordinateSystem::kField
                                      });
                builder.WptConstraint(i,
                                      trajopt::AngularVelocityConstraint{
                                              trajopt::IntervalSet1d{
                                                      waypoint.vehicle_velocity().omega(),
                                                      waypoint.vehicle_velocity().omega()
                                              }
                                      });
                break;
        }

        // Apply sample count
        if (i > 0) {
            unsigned int sample_count = waypoint.has_samples() ? waypoint.samples()
                                                               : guess_control_interval_count(waypoint,
                                                                                              waypoints.at(i - 1),
                                                                                              model);
            fmt::print("Adding sample count {} between waypoints {}-{}\n", sample_count, i - 1, i);
            control_intervals.push_back(sample_count);
        }
    }

    builder.ControlIntervalCounts(std::move(control_intervals));
}

void apply_segment_constraints(trajopt::SwervePathBuilder &builder, const std::vector<vts::Waypoint> &waypoints,
                               const vts::PathRequest &request) {
    int constraint_start_idx = -1;
    for (int i = 0; i < request.segments_size(); i++) {
        const vts::PathSegment &segment = request.segments(i);
        int constraint_end_idx = constraint_start_idx + segment.waypoints_size();
        if (constraint_start_idx < 0) {
            // First segment starts from the first waypoint since there is no segment behind it.
            constraint_start_idx = 0;
        }

        if (segment.has_max_velocity()) {
            fmt::print("Adding max velocity {} to segment {} (waypoints {}-{})\n", segment.max_velocity(),
                       i, constraint_start_idx, constraint_end_idx);
            builder.SgmtVelocityMagnitude(constraint_start_idx, constraint_end_idx,
                                          segment.max_velocity());
        }

        if (segment.has_max_omega()) {
            fmt::print("Adding max omega {} to segment {} (waypoints {}-{})\n", segment.max_omega(),
                       i, constraint_start_idx, constraint_end_idx);
            builder.SgmtConstraint(constraint_start_idx, constraint_end_idx,
                                   trajopt::AngularVelocityConstraint{segment.max_omega()});
        }

        if (segment.straight_line()) {
            double x1 = waypoints.at(constraint_start_idx).x();
            double x2 = waypoints.at(constraint_end_idx).x();
            double y1 = waypoints.at(constraint_start_idx).y();
            double y2 = waypoints.at(constraint_end_idx).y();
            double angle = atan2(y2 - y1, x2 - x1);
            fmt::print("Adding straight line constraint with angle {} to segment {} (waypoints {}-{})\n", angle,
                       i, constraint_start_idx, constraint_end_idx);
            builder.SgmtVelocityDirection(constraint_start_idx, constraint_end_idx, angle);
        }

        constraint_start_idx = constraint_end_idx;
    }
}

class VehicleTrajectoryService final
        : public vts::VehicleTrajectoryService::Service {
public:
    ::grpc::Status GenerateTrajectory(::grpc::ServerContext *context,
                                      const vts::PathRequest *request,
                                      vts::TrajectoryResponse *response) override {
        trajopt::SwervePathBuilder builder;
        builder.SetDrivetrain(create_drivetrain(request->model()));

        auto all_waypoints = add_waypoints(builder, *request);
        apply_waypoint_constraints(builder, all_waypoints, request->model());
        apply_segment_constraints(builder, all_waypoints, *request);

        try {
            fmt::print("Generating trajectory\n");
            auto solution = trajopt::OptimalTrajectoryGenerator::Generate(builder);
            if (solution.has_value()) {
                fmt::print("Generation finished\n");
                convert_solution(response->mutable_trajectory(), solution.value());
            } else {
                fmt::print("Generation failed: {}\n", std::string(solution.error()));
                response->mutable_error()->set_reason(std::string(solution.error()));
            }
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

    // This is a hack but who cares
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGKILL, signal_handler);

    server = std::move(builder.BuildAndStart());
    server->Wait();

    return 0;
}
