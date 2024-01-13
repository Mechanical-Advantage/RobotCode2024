package frc.robot.subsystems.kitbotshooter;

import org.littletonrobotics.junction.AutoLog;

public interface KitbotFlywheelIO {
  @AutoLog
  class KitbotFlywheelIOInputs {
    public double[] flywheelPositionRads = new double[] {};
    public double[] flywheelVelocityRadPerSec = new double[] {};
    public double[] flywheelAppliedVolts = new double[] {};
    public double[] flywheelCurrentAmps = new double[] {};
  }

  default void updateInputs(KitbotFlywheelIOInputs inputs) {}
  ;

  /** run to velocity */
  default void runVelocity(double velocityRadPerSec) {}

  /** run at voltage */
  default void runVolts(double volts) {}

  /** set brakeMode */
  default void setBrakeMode(boolean brake) {}

  default double getVelocityRPM() {
    return 0;
  }
  ;
}
