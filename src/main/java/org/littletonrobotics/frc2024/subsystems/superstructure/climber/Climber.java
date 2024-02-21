// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.climber;

import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevator;

public class Climber extends GenericSlamElevator {
  public Climber(ClimberIO io) {
    super("Climber", io, 12.0, 0.4, 0.1);
  }
}
