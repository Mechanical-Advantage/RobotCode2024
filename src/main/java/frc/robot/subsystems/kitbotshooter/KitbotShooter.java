package frc.robot.subsystems.kitbotshooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class KitbotShooter extends SubsystemBase {
  private static double flywheelIntakeVoltage = -5.0;
  private static double feederIntakeVoltage = -5.0;

  private FlywheelIO flywheelIO;
  private FeederIO feederIO;

  private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  private MODE currentMode = null;

  public KitbotShooter(FlywheelIO flywheelIO, FeederIO feederIO) {
    this.flywheelIO = flywheelIO;
    this.feederIO = feederIO;
  }

  @Override
  public void periodic() {
    // update inputs
    flywheelIO.updateInputs(flywheelInputs);
    feederIO.updateInputs(feederInputs);
    // process inputs
    Logger.processInputs("KitbotShooter/Flywheel", flywheelInputs);
    Logger.processInputs("KitbotShooter/Feeder", feederInputs);

    // check for current mode
    if (currentMode == null) {
      currentMode = DriverStation.isEnabled() ? MODE.ENABLED : MODE.DISABLED;
      flywheelIO.setBrakeMode(DriverStation.isEnabled());
      feederIO.setBrakeMode(DriverStation.isEnabled());
    }

    if (currentMode == MODE.DISABLED) {
      flywheelIO.setBrakeMode(false);
      feederIO.setBrakeMode(false);
    } else if (currentMode == MODE.ENABLED) {
      // other stuff
    } else if (currentMode == MODE.CHARACTERIZING) {

    }
  }

  /** Set input voltage for flywheel */
  public void runFlywheelVolts(double volts) {
    flywheelIO.runVolts(volts);
  }

  /** Set input voltage for feeder */
  public void runFeederVolts(double volts) {
    feederIO.runVolts(volts);
  }

  /** Run flywheel at velocity */
  public void runFlywheelVelocity(double rpm) {
    if (currentMode != MODE.ENABLED) return;
    double rps = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    flywheelIO.runVelocity(Units.rotationsToRadians(rps));
  }

  public void setCurrentMode(MODE mode) {
    currentMode = mode;
  }

  @AutoLogOutput(key = "KitbotShooter/FlywheelRPM")
  public double getFlywheelRPM() {
    return flywheelIO.getVelocityRPM();
  }

  @AutoLogOutput(key = "KitbotShooter/FlywheelVelocityRadPerSec")
  public double getFlywheelVelocityRadPerSec() {
    return Arrays.stream(flywheelInputs.flywheelVelocityRadPerSec).sum() / 2.0;
  }

  @AutoLogOutput(key = "KitbotShooter/FeederVelocityRadPerSec")
  public double getFeederVelocityRadPerSec() {
    return feederInputs.feederVelocityRadPerSec;
  }

  @AutoLogOutput(key = "KitbotShooter/FeederCurrent")
  public double getFeederCurrent() {
    return feederInputs.feederCurrentAmps;
  }

  public Command intakeCommand() {
    return Commands.startEnd(
            () -> {
              flywheelIO.runVolts(flywheelIntakeVoltage);
              feederIO.runVolts(feederIntakeVoltage);
            },
            () -> {
              flywheelIO.runVolts(0.0);
              feederIO.runVolts(0.0);
            });
  }

  public enum MODE {
    ENABLED,
    DISABLED,
    CHARACTERIZING;
  }
}
