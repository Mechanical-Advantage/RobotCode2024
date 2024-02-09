package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryGenerationHelpers {
    /**
     * Gets the pose of the state
     *
     * @param state The {@link VehicleState}
     * @return {@link Pose2d} of the state
     */
    public static Pose2d getPose(VehicleState state) {
        return new Pose2d(state.getX(), state.getY(), Rotation2d.fromRadians(state.getTheta()));
    }

    /**
     * Creates a {@link VehicleVelocityConstraint} from a {@link VehicleState}, useful for chaining a
     * generated trajectory into a new trajectory.
     *
     * @param state The {@link VehicleState}
     * @return The {@link VehicleVelocityConstraint} representing the velocity of the state.
     */
    public static VehicleVelocityConstraint createVelocityConstraint(VehicleState state) {
        return VehicleVelocityConstraint.newBuilder()
                .setVx(state.getVx())
                .setVy(state.getVy())
                .setOmega(state.getOmega())
                .build();
    }

    public static VehicleVelocityConstraint endVelocityConstraint(Trajectory trajectory) {
        return createVelocityConstraint(
                trajectory.getStates(trajectory.getStatesCount() - 1).getState());
    }

    /**
     * Adds a waypoint to an existing {@link PathSegment.Builder} which is a continuation of a
     * previously generated trajectory. The pose and velocity of the waypoint are set to the ending
     * pose and velocity of the trajectory.
     *
     * @param builder The {@link PathSegment.Builder}.
     * @param trajectory The generated trajectory.
     * @return The {@link PathSegment.Builder} with the new waypoint added.
     */
    public static PathSegment.Builder addContinuationWaypoint(
            PathSegment.Builder builder, Trajectory trajectory) {
        Pose2d endPose = getPose(trajectory.getStates(trajectory.getStatesCount() - 1).getState());
        return builder.addWaypoints(
                fromPose(Waypoint.newBuilder(), endPose)
                        .setVehicleVelocity(endVelocityConstraint(trajectory)));
    }

    /**
     * Adds a translation waypoint to an existing {@link PathSegment.Builder} without setting any
     * other constraints.
     *
     * @param builder The {@link PathSegment.Builder}.
     * @param translation The translation.
     * @return The {@link PathSegment.Builder} with the new waypoint added.
     */
    public static PathSegment.Builder addTranslationWaypoint(
            PathSegment.Builder builder, Translation2d translation) {
        return builder.addWaypoints(fromTranslation(Waypoint.newBuilder(), translation));
    }

    /**
     * Adds a waypoint to an existing {@link PathSegment.Builder} without setting any other
     * costraints.
     *
     * @param builder The {@link PathSegment.Builder}
     * @param pose The pose.
     * @return The {@link PathSegment.Builder} with the new waypoint added.
     */
    public static PathSegment.Builder addPoseWaypoint(PathSegment.Builder builder, Pose2d pose) {
        return builder.addWaypoints(fromPose(Waypoint.newBuilder(), pose));
    }

    /**
     * Adds a translation waypoint to an existing {@link PathSegment.Builder} without setting any
     * other constraints.
     *
     * @param builder The {@link PathSegment.Builder}.
     * @param translation The translation.
     * @return The {@link PathSegment.Builder} with the new waypoint added.
     */
    public static PathSegment.Builder addTranslationWaypoint(
            PathSegment.Builder builder, Translation2d translation, int samples) {
        return builder.addWaypoints(
                fromTranslation(Waypoint.newBuilder(), translation).setSamples(samples));
    }

    /**
     * Adds a waypoint to an existing {@link PathSegment.Builder} without setting any other
     * costraints.
     *
     * @param builder The {@link PathSegment.Builder}
     * @param pose The pose.
     * @return The {@link PathSegment.Builder} with the new waypoint added.
     */
    public static PathSegment.Builder addPoseWaypoint(
            PathSegment.Builder builder, Pose2d pose, int samples) {
        return builder.addWaypoints(fromPose(Waypoint.newBuilder(), pose).setSamples(samples));
    }

    /**
     * Adds {@link Translation2d} to an existing {@link Waypoint.Builder}
     *
     * @param builder The {@link Waypoint.Builder}
     * @param translation The translation.
     * @return The {@link Waypoint.Builder} with the translation added.
     */
    public static Waypoint.Builder fromTranslation(
            Waypoint.Builder builder, Translation2d translation) {
        return builder.setX(translation.getX()).setY(translation.getY());
    }

    /**
     * Adds {@link Pose2d} to an existing {@link Waypoint.Builder}
     *
     * @param builder The {@link Waypoint.Builder}
     * @param pose The pose.
     * @return The {@link Waypoint.Builder} with the pose added.
     */
    public static Waypoint.Builder fromPose(Waypoint.Builder builder, Pose2d pose) {
        return fromTranslation(builder, pose.getTranslation())
                .setHeadingConstraint(pose.getRotation().getRadians());
    }
}
