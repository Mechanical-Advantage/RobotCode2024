// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevatorIOSim;

public class ClimberIOSim extends GenericSlamElevatorIOSim implements ClimberIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
  private static final double maxLength = 42;

  public ClimberIOSim() {
    super(motorModel, maxLength);
  }
}
