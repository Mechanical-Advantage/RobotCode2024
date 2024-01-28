package frc.robot.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;

public interface Trajectory {
  double getDuration();

  Pose2d getStartPose();

  Pose2d[] getTrajectoryPoses();

  HolonomicDriveController.HolonomicDriveState getStartState();

  HolonomicDriveController.HolonomicDriveState getEndState();

  HolonomicDriveController.HolonomicDriveState sample(double timeSeconds);
}
