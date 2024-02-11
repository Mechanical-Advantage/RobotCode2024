package frc.robot.commands;

import static frc.robot.Constants.Mode.SIM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class TurnToSpeaker extends Command {
  private final Drive drive;
  private final PIDController controller;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double minVelocity = 0;
  private static double toleranceDegrees = 0;
  private DriverStation.Alliance alliance = null;

  /** Creates a new TurnToAngle. Turns to the specified rotation. */
  public TurnToSpeaker(Drive drive) {
    addRequirements(drive);
    this.drive = drive;

    switch (Constants.currentMode) {
      case REAL:
        kP = 0.018;
        kI = 0.0;
        kD = 0.0;
        minVelocity = 0.0;
        toleranceDegrees = 1.0;
        break;
      default: // for SIM
        kP = 0.1;
        kI = 0.0;
        kD = 0.001;
        minVelocity = 0.0;
        toleranceDegrees = 1.5;
        break;
    }

    controller = new PIDController(kP, kI, kD, 0.02);
    controller.setTolerance(toleranceDegrees);
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();

    if (DriverStation.getAlliance().isPresent()) {
      this.alliance = DriverStation.getAlliance().get();
    }

    if (alliance == DriverStation.Alliance.Red) {
      controller.setSetpoint(
          Math.toDegrees(
                  Math.atan2(
                      Constants.SPEAKER_Y - drive.getPose().getY(),
                      Constants.FIELD_LENGTH - drive.getPose().getX()))
              - (Constants.currentMode == SIM ? Drive.YAW_DISPLACEMENT : 0));
    } else {
      controller.setSetpoint(
          Math.toDegrees(
              Math.atan2(Constants.SPEAKER_Y - drive.getPose().getY(), -drive.getPose().getX())
                  - (Constants.currentMode == SIM ? Drive.YAW_DISPLACEMENT : 0)));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("TurnActive", true);
    Logger.recordOutput("Turn X Goal", controller.getSetpoint());
    // Update output speeds

    double angularSpeed = controller.calculate(drive.getGyroYawDegrees());

    if (Math.abs(angularSpeed) < minVelocity) {
      angularSpeed = Math.copySign(minVelocity, angularSpeed);
    }

    Logger.recordOutput("Angular Speed", angularSpeed);
    drive.driveVelocity(
        -angularSpeed * Drive.TRACK_WIDTH / 2.0, angularSpeed * Drive.TRACK_WIDTH / 2.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("TurnActive", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
