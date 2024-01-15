package frc.robot.commands;

import static frc.robot.util.trajectory.HolonomicDriveController.HolonomicDriveState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.HolonomicDriveController;
import frc.robot.util.trajectory.Trajectory;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory extends Command {
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("DriveTrajectory/DriveKp");
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("DriveTrajectory/DriveKd");
  private static final LoggedTunableNumber turnKp =
      new LoggedTunableNumber("DriveTrajectory/TurnKp");
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("DriveTrajectory/TurnKd");

  // initialize tunable numbers to constants
  static {
    driveKp.initDefault(DriveConstants.trajectoryConstants().drivekp());
    driveKd.initDefault(DriveConstants.trajectoryConstants().drivekd());
    turnKp.initDefault(DriveConstants.trajectoryConstants().turnkp());
    turnKd.initDefault(DriveConstants.trajectoryConstants().turnkd());
  }

  public static final PIDController linearController = new PIDController(0.0, 0.0, 0.0);
  public static final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);
  public static final HolonomicDriveController controller =
      new HolonomicDriveController(linearController, thetaController);

  private final Drive drive;
  private final Supplier<Trajectory> trajectorySupplier;
  private Trajectory trajectory;
  private final Timer timer = new Timer();

  public DriveTrajectory(Drive drive, Supplier<Trajectory> trajectorySupplier) {
    this.drive = drive;
    this.trajectorySupplier = trajectorySupplier;

    addRequirements(drive);
  }

  public DriveTrajectory(Drive drive, Trajectory trajectory) {
    this(drive, () -> trajectory);
  }

  @Override
  public void initialize() {
    // Log trajectory
    trajectory = trajectorySupplier.get();
    Logger.recordOutput("Odometry/Trajectory", trajectory.getTrajectoryPoses());

    // Reset all controllers
    timer.reset();
    timer.start();
    linearController.reset();
    thetaController.reset();

    // Reset PID gains
    linearController.setP(driveKp.get());
    linearController.setD(driveKd.get());
    thetaController.setP(turnKp.get());
    thetaController.setD(turnKd.get());
  }

  @Override
  public void execute() {
    if (driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || turnKp.hasChanged(hashCode())
        || turnKd.hasChanged(hashCode())) {
      linearController.setP(driveKp.get());
      linearController.setD(driveKd.get());
      thetaController.setP(turnKp.get());
      thetaController.setD(turnKd.get());
    }

    // sample and maybe flip trajectory state
    HolonomicDriveState referenceState = trajectory.sample(timer.get());
    referenceState = AllianceFlipUtil.apply(referenceState);

    Pose2d referencePose = referenceState.pose();
    Logger.recordOutput("Odometry/TrajectorySetpoint", referencePose);

    ChassisSpeeds outputSpeeds = controller.calculate(drive.getPose(), referenceState);
    drive.runVelocity(outputSpeeds);

    Logger.recordOutput(
        "Odometry/TrajectoryTranslationErrorMeters",
        controller.getPoseError().getTranslation().getNorm());
    Logger.recordOutput(
        "Odometry/TrajectoryRotationErrorRadians", controller.getRotationError().getRadians());
    Logger.recordOutput(
        "Drive/autoSpeeds",
        new double[] {
          outputSpeeds.vxMetersPerSecond,
          outputSpeeds.vyMetersPerSecond,
          outputSpeeds.omegaRadiansPerSecond
        });
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getDuration());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("Odometry/Trajectory", new Pose2d[] {});
    Logger.recordOutput("Odometry/TrajectorySetpoint", new double[] {});
  }
}
