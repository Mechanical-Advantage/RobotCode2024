package frc.robot.commands.auto;

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
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.Trajectory;
import java.io.File;

public class TestAutos {
  private static final LoggedTunableNumber intakeDistance =
      new LoggedTunableNumber(
          "Auto/intakeDistance", DriveConstants.drivetrainConfig.trackwidthX() + 0.1);

  // bottom to top
  private static AutoCommands.CircularRegion spikeIntakeRegion(int i) {
    return new AutoCommands.CircularRegion(
        FieldConstants.StagingLocations.spikeTranslations[i], intakeDistance.get());
  }

  private static AutoCommands.CircularRegion centerlineIntakeRegion(int i) {
    return new AutoCommands.CircularRegion(
        FieldConstants.StagingLocations.centerlineTranslations[i], intakeDistance.get());
  }

  private static final RobotState robotState = RobotState.getInstance();

  public static Command davisAutoDefensive(Drive drive, Intake intake) {
    Trajectory trajectory = getTrajectory("DavisAutoDefensive.traj");

    Command sequenceIntake =
        Commands.sequence(
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(spikeIntakeRegion(2))),
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(centerlineIntakeRegion(4))),
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(spikeIntakeRegion(1))));

    var secondShot =
        new AutoCommands.RectangularRegion(
            new Translation2d(3.7, 7.1), 0.2, DriveConstants.drivetrainConfig.trackwidthY());
    var thirdShot =
        new AutoCommands.RectangularRegion(
            new Translation2d(4.8, 6.2), 0.2, DriveConstants.drivetrainConfig.trackwidthY());

    Command sequenceShots =
        Commands.sequence(
            moveWhileShooting(drive),
            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(secondShot)),
            moveWhileShooting(drive),
            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(thirdShot)),
            moveWhileShooting(drive),
            Commands.waitUntil(() -> !drive.getMotionPlanner().followingTrajectory()),
            moveWhileShooting(drive));

    return Commands.sequence(
        resetPoseCommand(trajectory.getStartPose()),
        Commands.waitSeconds(1.0), // Initial rev time for first shot
        drive.followTrajectoryCommand(trajectory),
        Commands.parallel(sequenceIntake, sequenceShots));
  }

  public static Command davisAuto(Drive drive, Intake intake) {
    Trajectory trajectory = getTrajectory("DavisAutoOG.traj");

    Command sequenceIntake =
        Commands.sequence(
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(spikeIntakeRegion(2))),
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(spikeIntakeRegion(1))),
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(centerlineIntakeRegion(4))));

    var secondShot =
        new AutoCommands.RectangularRegion(
            new Translation2d(2.57, 7.3), DriveConstants.drivetrainConfig.trackwidthX(), 0.2);
    var thirdShot =
        new AutoCommands.RectangularRegion(
            new Translation2d(3.7, 5.8), 0.2, DriveConstants.drivetrainConfig.trackwidthY());
    var fourthShot =
        new AutoCommands.RectangularRegion(
            new Translation2d(4.34, 7.20), 0.2, DriveConstants.drivetrainConfig.trackwidthY());

    Command sequenceShots =
        Commands.sequence(
            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(secondShot)),
            moveWhileShooting(drive),
            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(thirdShot)),
            moveWhileShooting(drive),
            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(fourthShot)),
            moveWhileShooting(drive));

    return Commands.sequence(
        resetPoseCommand(trajectory.getStartPose()),
        Commands.waitSeconds(1.2), // Preload shot
        drive.followTrajectoryCommand(trajectory), // start trajectory
        Commands.parallel(sequenceIntake, sequenceShots));
  }

  private static Command resetPoseCommand(Pose2d pose) {
    return Commands.runOnce(() -> robotState.resetPose(AllianceFlipUtil.apply(pose)));
  }

  private static Command moveWhileShooting(Drive drive) {
    return drive
        .setHeadingCommand(TestAutos::getShootHeading)
        .andThen(Commands.waitSeconds(0.7))
        .andThen(drive.disableHeadingCommand());
  }

  private static Rotation2d getShootHeading() {
    Twist2d fieldVel = RobotState.getInstance().fieldVelocity();
    return ShotCalculator.calculate(
            AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation()),
            RobotState.getInstance().getEstimatedPose().getTranslation(),
            new Translation2d(fieldVel.dx, fieldVel.dy))
        .goalHeading();
  }

  private static Trajectory getTrajectory(String fileName) {
    File trajFile = new File(Filesystem.getDeployDirectory(), "/choreo/" + fileName);
    return generate(trajFile).orElseThrow();
  }
}
