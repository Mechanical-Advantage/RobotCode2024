package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double leftFlywheelPositionRotations = 0.0;
    public double leftFlywheelVelocityRPM = 0.0;
    public double leftFlywheelAppliedVolts = 0.0;
    public double leftFlywheelOutputCurrent = 0.0;

    public double rightFlywheelPositionRotations = 0.0;
    public double rightFlywheelVelocityRPM = 0.0;
    public double rightFlywheelAppliedVolts = 0.0;
    public double rightFlywheelOutputCurrent = 0.0;

    public double feederVelocityRPM = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederOutputCurrent = 0.0;
  }

  /** Update inputs */
  default void updateInputs(ShooterIOInputs inputs) {}

  default void setLeftRPM(double rpm) {}

  default void setRightRPM(double rpm) {}

  default void setRPM(double leftRpm, double rightRpm) {
    setLeftRPM(leftRpm);
    setRightRPM(rightRpm);
  }

  default void setFeederVoltage(double volts) {}

  default void setLeftBrakeMode(boolean enabled) {}

  default void setRightBrakeMode(boolean enabled) {}

  default void setShooterBrakeMode(boolean enabled) {
    setLeftBrakeMode(enabled);
    setRightBrakeMode(enabled);
  }

  default void setFeederBrakeMode(boolean enabled) {}

  default void setLeftCharacterizationVoltage(double volts) {}

  default void setRightCharacterizationVoltage(double volts) {}

  default void setLeftPID(double p, double i, double d) {}

  default void setLeftFF(double s, double v, double a) {}

  default void setRightPID(double p, double i, double d) {}

  default void setRightFF(double s, double v, double a) {}

  default void stop() {}
}
