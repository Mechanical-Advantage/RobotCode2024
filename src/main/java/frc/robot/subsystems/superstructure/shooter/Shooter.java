package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber feedVolts =
      new LoggedTunableNumber("Shooter/FeedVolts", 6.0);
  private static final LoggedTunableNumber leftkP =
      new LoggedTunableNumber("Shooter/leftkP", leftFlywheelConstants.kP());
  private static final LoggedTunableNumber leftkI =
      new LoggedTunableNumber("Shooter/leftkI", leftFlywheelConstants.kI());
  private static final LoggedTunableNumber leftkD =
      new LoggedTunableNumber("Shooter/leftkD", leftFlywheelConstants.kD());
  private static final LoggedTunableNumber leftkS =
      new LoggedTunableNumber("Shooter/leftkS", leftFlywheelConstants.kS());
  private static final LoggedTunableNumber leftkV =
      new LoggedTunableNumber("Shooter/leftkV", leftFlywheelConstants.kV());
  private static final LoggedTunableNumber leftkA =
      new LoggedTunableNumber("Shooter/leftkA", leftFlywheelConstants.kA());
  private static final LoggedTunableNumber rightkP =
      new LoggedTunableNumber("Shooter/rightkP", rightFlywheelConstants.kP());
  private static final LoggedTunableNumber rightkI =
      new LoggedTunableNumber("Shooter/rightkI", rightFlywheelConstants.kI());
  private static final LoggedTunableNumber rightkD =
      new LoggedTunableNumber("Shooter/rightkD", rightFlywheelConstants.kD());
  private static final LoggedTunableNumber rightkS =
      new LoggedTunableNumber("Shooter/rightkS", rightFlywheelConstants.kS());
  private static final LoggedTunableNumber rightkV =
      new LoggedTunableNumber("Shooter/rightkV", rightFlywheelConstants.kV());
  private static final LoggedTunableNumber rightkA =
      new LoggedTunableNumber("Shooter/rightkA", rightFlywheelConstants.kA());
  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM", shooterToleranceRPM);
  private static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Shooter/IntakeVolts", -5.0);
  private final LoggedDashboardNumber leftSpeedRpm =
      new LoggedDashboardNumber("Left Speed RPM", 6000);
  private final LoggedDashboardNumber rightSpeedRpm =
      new LoggedDashboardNumber("Right Speed RPM", 4000);

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private double characterizationVolts = 0.0;
  private boolean characterizing = false;
  private boolean shooting = false;
  private boolean intaking = false;

  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    shooterIO = io;
    shooterIO.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get());
    shooterIO.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get());
    shooterIO.setRightFF(rightkS.get(), rightkV.get(), rightkA.get());
    shooterIO.setRightPID(rightkP.get(), rightkI.get(), rightkD.get());
  }

  @Override
  public void periodic() {
    // check controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get()),
        leftkP,
        leftkI,
        leftkD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get()),
        leftkS,
        leftkV,
        leftkA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setRightPID(rightkP.get(), rightkI.get(), rightkD.get()),
        rightkP,
        rightkI,
        rightkD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> shooterIO.setRightFF(rightkS.get(), rightkV.get(), rightkA.get()),
        rightkS,
        rightkV,
        rightkA);

    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    if (DriverStation.isDisabled()) {
      shooterIO.stop();
    } else {
      if (!characterizing) {
        if (shooting) {
          shooterIO.setRPM(leftSpeedRpm.get(), rightSpeedRpm.get());
          double feederSetpointVolts = 0.0;
          if (leftSpeedRpm.get() > 0 && rightSpeedRpm.get() > 0 && atSetpoint()) {
            feederSetpointVolts = feedVolts.get();
          } else {
            feederSetpointVolts = 0;
          }
          shooterIO.setFeederVoltage(feederSetpointVolts);
        } else if (intaking) {
          shooterIO.setLeftCharacterizationVoltage(intakeVolts.get());
          shooterIO.setRightCharacterizationVoltage(intakeVolts.get());
          shooterIO.setFeederVoltage(intakeVolts.get());
        } else {
          shooterIO.stop();
        }
      }
    }

    Logger.recordOutput("Shooter/LeftRPM", shooterInputs.leftFlywheelVelocityRPM);
    Logger.recordOutput("Shooter/RightRPM", shooterInputs.rightFlywheelVelocityRPM);
    Logger.recordOutput("Shooter/FeederRPM", shooterInputs.feederVelocityRPM);
  }

  public void setIntaking() {
    characterizing = false;
    shooting = false;
    intaking = true;
  }

  public void setShooting() {
    characterizing = false;
    shooting = true;
    intaking = false;
  }

  public void stop() {
    characterizing = false;
    shooting = false;
    intaking = false;
  }

  public void runLeftCharacterizationVolts(double volts) {
    characterizing = true;
    shooterIO.setLeftCharacterizationVoltage(volts);
  }

  public void runRightCharacterizationVolts(double volts) {
    characterizing = true;
    shooterIO.setRightCharacterizationVoltage(volts);
  }

  public double getLeftCharacterizationVelocity() {
    return shooterInputs.leftFlywheelVelocityRPM;
  }

  public double getRightCharacterizationVelocity() {
    return shooterInputs.rightFlywheelVelocityRPM;
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(shooterInputs.leftFlywheelVelocityRPM - leftSpeedRpm.get())
            <= shooterTolerance.get()
        && Math.abs(shooterInputs.rightFlywheelVelocityRPM - rightSpeedRpm.get())
            <= shooterTolerance.get();
  }
}
