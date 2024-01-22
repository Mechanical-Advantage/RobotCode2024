package frc.robot.commands;

import static frc.robot.util.trajectory.ChoreoTrajectoryReader.generate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveMotionPlanner;
import frc.robot.util.trajectory.Trajectory;
import java.io.File;

public class TestAutos {
  public static Command threeSpikeSmooth(Drive drive) {
    File trajFile = new File(Filesystem.getDeployDirectory(), "/choreo/3SpikeSmooth.traj");
    Trajectory trajectory = generate(trajFile).orElseThrow();
    DriveMotionPlanner motionPlanner = drive.getMotionPlanner();

    // Waiting inbetween shots is to fake intaking
    // Would not be time based on real robot
    return Commands.sequence(
        reset(trajectory.getStartPose(), drive),
        Commands.waitSeconds(1.5), // Shoot first 1.5 secs
        Commands.runOnce(() -> motionPlanner.setTrajectory(trajectory)),
        Commands.waitSeconds(0.7), // 2.2 secs
        moveWhileShooting(motionPlanner), // 2.9 secs
        Commands.waitSeconds(0.3), // 3.2 secs
        moveWhileShooting(motionPlanner), // 3.9 secs
        Commands.waitSeconds(0.61), // 4.51 secs
        moveWhileShooting(motionPlanner)); // 5.21 secs
  }

  private static Command moveWhileShooting(DriveMotionPlanner motionPlanner) {
    return Commands.runOnce(() -> motionPlanner.setHeadingSupplier(TestAutos::getShootHeading))
        .andThen(
            Commands.waitSeconds(0.7)
                .andThen(
                    Commands.runOnce(motionPlanner::disableHeadingSupplier))); // Time to shoot ig
  }

  private static Rotation2d getShootHeading() {
    return FieldConstants.Speaker.centerSpeakerOpening
        .getTranslation()
        .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
        .getAngle()
        .plus(Rotation2d.fromRadians(Math.PI));
  }

  private static Command reset(Pose2d pose, Drive drive) {
    return Commands.runOnce(
        () ->
            RobotState.getInstance()
                .resetPose(pose, drive.getWheelPositions(), drive.getGyroYaw()));
  }
}
