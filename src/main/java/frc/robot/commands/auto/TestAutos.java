package frc.robot.commands.auto;

import static frc.robot.util.trajectory.ChoreoTrajectoryReader.generate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.Trajectory;
import java.io.File;

public class TestAutos {
  private static final LoggedTunableNumber intakeDistance =
      new LoggedTunableNumber(
          "Auto/intakeDistance", DriveConstants.driveConfig.trackwidthX() + 0.1);

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

  public static Command fourNoteDavis(Drive drive, Intake intake) {
    Trajectory trajectory = getTrajectory("4NoteDavis.traj");

    Command sequenceIntake =
        Commands.sequence(
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(spikeIntakeRegion(2))),
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(spikeIntakeRegion(1))),
            AutoCommands.intakeWhileInRegion(
                intake, () -> AllianceFlipUtil.apply(centerlineIntakeRegion(4))));

    var secondShotRegion =
        new AutoCommands.RectangularRegion(new Translation2d(2.11, 6.37), 0.5, 0.1);
    var thirdShotRegion =
        new AutoCommands.RectangularRegion(new Translation2d(3.83, 5.99), 0.1, 0.5);
    var fourthShotRegion =
        new AutoCommands.RectangularRegion(new Translation2d(4.08, 7.18), 0.1, 0.5);
    //    Command sequenceShots =
    //        Commands.sequence(
    //            AutoCommands.moveWhileShooting(drive),
    //            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(secondShotRegion)),
    //            AutoCommands.moveWhileShooting(drive),
    //            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(thirdShotRegion)),
    //            AutoCommands.moveWhileShooting(drive),
    //            AutoCommands.waitForRegion(() -> AllianceFlipUtil.apply(fourthShotRegion)),
    //            AutoCommands.moveWhileShooting(drive));

    return Commands.sequence(
        resetPoseCommand(trajectory.getStartPose()),
        Commands.parallel(drive.followTrajectory(trajectory), sequenceIntake));
  }

  private static Command resetPoseCommand(Pose2d pose) {
    return Commands.runOnce(() -> robotState.resetPose(AllianceFlipUtil.apply(pose)));
  }

  private static Trajectory getTrajectory(String fileName) {
    File trajFile = new File(Filesystem.getDeployDirectory(), "/choreo/" + fileName);
    return generate(trajFile).orElseThrow();
  }
}
