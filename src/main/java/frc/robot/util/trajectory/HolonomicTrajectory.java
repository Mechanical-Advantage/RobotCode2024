package frc.robot.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;

public interface HolonomicTrajectory {
  double getDuration();

  Pose2d getStartPose();

  Pose2d[] getTrajectoryPoses();

  State getStartState();

  State getEndState();

  State sample(double timeSeconds);

  record State(
      double timeSeconds,
      Pose2d pose,
      double velocityX,
      double velocityY,
      double angularVelocity) {}
}
