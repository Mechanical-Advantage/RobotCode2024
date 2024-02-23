// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator;

import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevatorIOKrakenFOC;

public class BackpackActuatorIOKrakenFOC extends GenericSlamElevatorIOKrakenFOC
    implements BackpackActuatorIO {
  private static final int id = 0;
  private static final String bus = "rio";
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final double reduction = 32.0 / 10.0;

  public BackpackActuatorIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, reduction);
  }
}
