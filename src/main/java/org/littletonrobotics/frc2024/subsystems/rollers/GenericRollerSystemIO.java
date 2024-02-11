package org.littletonrobotics.frc2024.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSystemIO {
  @AutoLog
  abstract class GenericRollerSystemIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double outputCurrent = 0.0;
  }

  default void updateInputs(GenericRollerSystemIOInputs inputs) {}

  /** Run feeder at volts */
  default void runVolts(double volts) {}

  /** Stop feeder */
  default void stop() {}
}
