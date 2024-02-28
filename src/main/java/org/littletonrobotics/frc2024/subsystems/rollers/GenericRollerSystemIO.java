// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSystemIO {
  @AutoLog
  abstract class GenericRollerSystemIOInputs {
    public boolean connected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(GenericRollerSystemIOInputs inputs) {}

  /** Run feeder at volts */
  default void runVolts(double volts) {}

  /** Stop feeder */
  default void stop() {}
}
