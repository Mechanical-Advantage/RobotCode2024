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

  private KitbotFlywheelIO kitbotFlywheelIO;
  private KitbotFeederIO kitbotFeederIO;

  private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  private Mode currentMode = null;

  public KitbotShooter(KitbotFlywheelIO kitbotFlywheelIO, KitbotFeederIO kitbotFeederIO) {
    this.kitbotFlywheelIO = kitbotFlywheelIO;
    this.kitbotFeederIO = kitbotFeederIO;
  }

  @Override
  public void periodic() {
    // update inputs
    kitbotFlywheelIO.updateInputs(flywheelInputs);
    kitbotFeederIO.updateInputs(feederInputs);
    // process inputs
    Logger.processInputs("KitbotShooter/Flywheel", flywheelInputs);
    Logger.processInputs("KitbotShooter/Feeder", feederInputs);

    // check for current mode
    if (currentMode == null) {
      currentMode = DriverStation.isEnabled() ? Mode.ENABLED : Mode.DISABLED;
      kitbotFlywheelIO.setBrakeMode(DriverStation.isEnabled());
      kitbotFeederIO.setBrakeMode(DriverStation.isEnabled());
    }

    if (currentMode == Mode.DISABLED) {
      kitbotFlywheelIO.setBrakeMode(false);
      kitbotFeederIO.setBrakeMode(false);
    } else if (currentMode == Mode.ENABLED) {
      // other stuff
    } else if (currentMode == Mode.CHARACTERIZING) {

    }
  }

  /** Set input voltage for flywheel */
  public void runFlywheelVolts(double volts) {
    kitbotFlywheelIO.runVolts(volts);
  }

  /** Set input voltage for feeder */
  public void runFeederVolts(double volts) {
    kitbotFeederIO.runVolts(volts);
  }

  /** Run flywheel at velocity */
  public void runFlywheelVelocity(double rpm) {
    if (currentMode != Mode.ENABLED) return;
    double rps = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    kitbotFlywheelIO.runVelocity(Units.rotationsToRadians(rps));
  }

  public void setCurrentMode(Mode mode) {
    currentMode = mode;
  }

  @AutoLogOutput(key = "KitbotShooter/FlywheelRPM")
  public double getFlywheelRPM() {
    return kitbotFlywheelIO.getVelocityRPM();
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
          kitbotFlywheelIO.runVolts(flywheelIntakeVoltage);
          kitbotFeederIO.runVolts(feederIntakeVoltage);
        },
        () -> {
          kitbotFlywheelIO.runVolts(0.0);
          kitbotFeederIO.runVolts(0.0);
        });
  }

  public enum Mode {
    ENABLED,
    DISABLED,
    CHARACTERIZING;
  }
}
