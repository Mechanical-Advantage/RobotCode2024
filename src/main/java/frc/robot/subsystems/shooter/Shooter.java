package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  private static LoggedTunableNumber feedVolts = new LoggedTunableNumber("Shooter/FeedVolts", 6.0);
  private static LoggedTunableNumber leftkP =
      new LoggedTunableNumber("Shooter/leftkP", ShooterConstants.leftkP);
  private static LoggedTunableNumber leftkI =
      new LoggedTunableNumber("Shooter/leftkI", ShooterConstants.leftkI);
  private static LoggedTunableNumber leftkD =
      new LoggedTunableNumber("Shooter/leftkD", ShooterConstants.leftkD);
  private static LoggedTunableNumber leftkS =
      new LoggedTunableNumber("Shooter/leftkS", ShooterConstants.leftkS);
  private static LoggedTunableNumber leftkV =
      new LoggedTunableNumber("Shooter/leftkV", ShooterConstants.leftkV);
  private static LoggedTunableNumber leftkA =
      new LoggedTunableNumber("Shooter/leftkA", ShooterConstants.leftkA);
  private static LoggedTunableNumber rightkP =
      new LoggedTunableNumber("Shooter/rightkP", ShooterConstants.rightkP);
  private static LoggedTunableNumber rightkI =
      new LoggedTunableNumber("Shooter/rightkI", ShooterConstants.rightkI);
  private static LoggedTunableNumber rightkD =
      new LoggedTunableNumber("Shooter/rightkD", ShooterConstants.rightkD);
  private static LoggedTunableNumber rightkS =
      new LoggedTunableNumber("Shooter/rightkS", ShooterConstants.rightkS);
  private static LoggedTunableNumber rightkV =
      new LoggedTunableNumber("Shooter/rightkV", ShooterConstants.rightkV);
  private static LoggedTunableNumber rightkA =
      new LoggedTunableNumber("Shooter/rightkA", ShooterConstants.rightkA);
  private static LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM", ShooterConstants.shooterToleranceRPM);

  private final LoggedDashboardNumber leftSpeedRpm =
      new LoggedDashboardNumber("Left Speed RPM", 6000);
  private final LoggedDashboardNumber rightSpeedRpm =
      new LoggedDashboardNumber("Right Speed RPM", 4000);

  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private double feederSetpointVolts = 0.0;

  private boolean characterizing = false;

  public Shooter(ShooterIO io) {
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
        shooterIO.setRPM(leftSpeedRpm.get(), rightSpeedRpm.get());
        if (leftSpeedRpm.get() > 0 && rightSpeedRpm.get() > 0 && atSetpoint()) {
          feederSetpointVolts = feedVolts.get();
        } else {
          feederSetpointVolts = 0;
        }
        shooterIO.setFeederVoltage(feederSetpointVolts);
      }
    }

    Logger.recordOutput("Shooter/LeftRPM", shooterInputs.leftFlywheelVelocityRPM);
    Logger.recordOutput("Shooter/RightRPM", shooterInputs.rightFlywheelVelocityRPM);
    Logger.recordOutput("Shooter/FeederRPM", shooterInputs.feederVelocityRPM);
  }

  public void runLeftCharacterizationVolts(double volts) {
    shooterIO.setLeftCharacterizationVoltage(volts);
  }

  public void runRightCharacterizationVolts(double volts) {
    shooterIO.setRightCharacterizationVoltage(volts);
  }

  public double getLeftCharacterizationVelocity() {
    return shooterInputs.leftFlywheelVelocityRPM;
  }

  public void setCharacterizing(boolean characterizing) {
    this.characterizing = characterizing;
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
