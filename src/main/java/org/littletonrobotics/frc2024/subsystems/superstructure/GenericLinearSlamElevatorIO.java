// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface GenericLinearSlamElevatorIO {
  @AutoLog
  class GenericLinearSlamElevatorIOInputs {
    public double positionRads = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
  }
}
