package frc.robot.commands;

import static frc.robot.util.trajectory.ChoreoTrajectoryReader.generate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveMotionPlanner;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.trajectory.Trajectory;
import java.io.File;

public class TestAutos {
  private static final RobotState robotState = RobotState.getInstance();

  public static Command fiveNoteLeavePodiumSpike(Drive drive) {
    Trajectory trajectory = getTrajectory("FiveNoteLeavePodiumSpike.traj");
    DriveMotionPlanner driveMotionPlanner = drive.getMotionPlanner();

    return Commands.sequence(
        reset(trajectory.getStartPose()),
        Commands.waitSeconds(1.0), // Initial rev time
        Commands.runOnce(() -> driveMotionPlanner.setTrajectory(trajectory)),
        moveWhileShooting(driveMotionPlanner), // 0.7 secs
        Commands.waitSeconds(0.8), // 1.5 secs
        moveWhileShooting(driveMotionPlanner), // 2.2 secs
        Commands.waitSeconds(1.2), // 3.4 secs
        moveWhileShooting(driveMotionPlanner), // 4.1 secs
        Commands.waitSeconds(2.85), // 6.95 secs
        moveWhileShooting(driveMotionPlanner), // 7.65 secs
        Commands.waitSeconds(2.35), // 10 secs
        moveWhileShooting(driveMotionPlanner)); // Finish path with last shot
  }

  public static Command davisAuto(Drive drive) {
    Trajectory trajectory = getTrajectory("DavisAutoPossible.traj");
    DriveMotionPlanner driveMotionPlanner = drive.getMotionPlanner();

    return Commands.sequence(
        reset(trajectory.getStartPose()),
        Commands.waitSeconds(1.2), // Preload shot
        Commands.runOnce(() -> driveMotionPlanner.setTrajectory(trajectory)), // start trajectory
        Commands.waitSeconds(1.0), // 1.0 secs
        moveWhileShooting(driveMotionPlanner), // 1.7 secs
        Commands.waitSeconds(1.6), // 3.3 secs
        moveWhileShooting(driveMotionPlanner), // 4.0 secs
        Commands.waitSeconds(3.0), // 7.0 secs
        moveWhileShooting(driveMotionPlanner) // 7.7 secs + finish rest of trajectory
        );
  }

  private static Command moveWhileShooting(DriveMotionPlanner driveMotionPlanner) {
    return Commands.runOnce(() -> driveMotionPlanner.setHeadingSupplier(TestAutos::getShootHeading))
        .andThen(
            Commands.waitSeconds(0.7) // Estimated shoot time
                .andThen(Commands.runOnce(driveMotionPlanner::disableHeadingSupplier)));
  }

  private static Rotation2d getShootHeading() {
    Twist2d fieldVel = RobotState.getInstance().fieldVelocity();
    return ShotCalculator.calculate(
            AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation()),
            RobotState.getInstance().getEstimatedPose().getTranslation(),
            new Translation2d(fieldVel.dx, fieldVel.dy))
        .goalHeading();
  }

  private static Command reset(Pose2d pose) {
    return Commands.runOnce(() -> robotState.resetPose(AllianceFlipUtil.apply(pose)));
  }

  private static Trajectory getTrajectory(String fileName) {
    File trajFile = new File(Filesystem.getDeployDirectory(), "/choreo/" + fileName);
    return generate(trajFile).orElseThrow();
  }
}
