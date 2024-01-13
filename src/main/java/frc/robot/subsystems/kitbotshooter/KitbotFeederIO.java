package frc.robot.subsystems.kitbotshooter;

import org.littletonrobotics.junction.AutoLog;

public interface KitbotFeederIO {
  @AutoLog
  class FeederIOInputs {
    public double feederPositionRads = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  default void updateInputs(FeederIOInputs inputs) {}
  ;

  /** run at voltage */
  default void runVolts(double volts) {}

  /** set brakeMode */
  default void setBrakeMode(boolean brake) {}

  default double getVelocityRPM() {
    return 0;
  }
}
