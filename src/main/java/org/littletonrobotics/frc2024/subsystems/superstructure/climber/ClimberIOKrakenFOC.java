// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.climber;

import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevatorIOKrakenFOC;

public class ClimberIOKrakenFOC extends GenericSlamElevatorIOKrakenFOC implements ClimberIO {
  private static final int id = 5;
  private static final String bus = "rio";
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final double reduction = 60.0 / 1.0;

  public ClimberIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, reduction);
  }
}
