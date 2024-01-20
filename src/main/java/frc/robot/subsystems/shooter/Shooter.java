package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  private static LoggedTunableNumber feedVolts = new LoggedTunableNumber("Shooter/FeedVolts", 6.0);
  private static LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
  private static LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI");
  private static LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");
  private static LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
  private static LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");
  private static LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA");
  private static LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM");

  private final LoggedDashboardNumber leftSpeedRpm =
      new LoggedDashboardNumber("Left Speed RPM", 1500.0);
  private final LoggedDashboardNumber rightSpeedRpm =
      new LoggedDashboardNumber("Right Speed RPM", 1500.0);

  static {
    kP.initDefault(ShooterConstants.kP);
    kI.initDefault(ShooterConstants.kI);
    kD.initDefault(ShooterConstants.kD);
    kS.initDefault(ShooterConstants.kS);
    kV.initDefault(ShooterConstants.kV);
    kA.initDefault(ShooterConstants.kA);
    shooterTolerance.initDefault(ShooterConstants.shooterToleranceRPM);
  }

  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private double feederSetpointVolts = 0.0;

  private boolean characterizing = false;

  public Shooter(ShooterIO io) {
    shooterIO = io;
    shooterIO.setPID(kP.get(), kI.get(), kD.get());
    shooterIO.setFF(kS.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    // check controllers
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode()))
      shooterIO.setPID(kP.get(), kI.get(), kD.get());
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()))
      shooterIO.setFF(kS.get(), kV.get(), kA.get());

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
