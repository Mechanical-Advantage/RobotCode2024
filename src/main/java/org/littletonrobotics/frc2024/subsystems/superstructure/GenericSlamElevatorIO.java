// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface GenericSlamElevatorIO {
  @AutoLog
  class GenericSlamElevatorIOInputs {
    public boolean motorConnected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Update the inputs. */
  default void updateInputs(GenericSlamElevatorIOInputs inputs) {}

  /** Run slam elevator at amps */
  default void runCurrent(double amps) {}

  /** Stop slam elevator */
  default void stop() {}

  /** Enable or disable brake mode on the elevator motor. */
  default void setBrakeMode(boolean enable) {}
}
