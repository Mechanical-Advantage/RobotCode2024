package frc.robot.commands;

import static frc.robot.Constants.Mode.REAL;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Turn extends Command {
  private final Drive drive;
  private Rotation2d setpoint;
  private final boolean relative;

  private final PIDController controller;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double minVelocity = 0;
  private static double toleranceDegrees = 0;

  /** Creates a new TurnToAngle. Turns to the specified rotation. */
  public Turn(Drive drive, Rotation2d setpoint, boolean relative) {
    addRequirements(drive);
    this.drive = drive;
    this.setpoint = Rotation2d.fromDegrees(setpoint.getDegrees());
    this.relative = relative;

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
    Logger.recordOutput("Turn Input", setpoint.getDegrees());
    if (relative) {
      controller.setSetpoint(
          Rotation2d.fromDegrees(drive.getGyroYawDegrees()).rotateBy(setpoint).getDegrees());
      setpoint = Rotation2d.fromDegrees((drive.getGyroYawDegrees()) + setpoint.getDegrees());
    } else {
      controller.setSetpoint(setpoint.getDegrees());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("TurnActive", true);
    Logger.recordOutput("Turn X Goal", setpoint.getDegrees());
    /*Units.radiansToDegrees(
    (Math.atan2(Constants.SPEAKER_Y - drive.getPose().getY(), drive.getPose().getX()))));*/
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
