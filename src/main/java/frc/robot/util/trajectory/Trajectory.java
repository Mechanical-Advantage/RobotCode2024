package frc.robot.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;

public interface Trajectory {
  double getDuration();

  Pose2d startPose();

  Pose2d[] getTrajectoryPoses();

  HolonomicDriveController.HolonomicDriveState sample(double timeSeconds);
}
