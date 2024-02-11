package org.littletonrobotics.frc2024.subsystems.superstructure.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  @AutoLog
  class FlywheelsIOInputs {
    public double leftPositionRads = 0.0;
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftOutputCurrent = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightOutputCurrent = 0.0;
    public double rightTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double leftVolts, double rightVolts) {}

  /** Stop both flywheels */
  default void stop() {}

  /** Run left and right flywheels at velocity in rpm */
  default void runVelocity(double leftRpm, double rightRpm) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run left flywheels at voltage */
  default void runCharacterizationLeftVolts(double volts) {}

  /** Run right flywheels at voltage */
  default void runCharacterizationRightVolts(double volts) {}
}
