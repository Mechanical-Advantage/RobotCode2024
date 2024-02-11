package org.littletonrobotics.frc2024.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersSensorsIO {
  @AutoLog
  class RollersSensorsIOInputs {
    boolean shooterStaged;
    boolean backbackStaged;
    boolean indexerCleared;
  }

  default void updateInputs(RollersSensorsIOInputs inputs) {}
}
