// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOKrakenFOC;

public class BackpackIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements BackpackIO {
  private static final int id = 2;
  private static final String bus = "rio";
  private static final int currentLimitAmps = 50;
  private static final boolean invert = false;
  private static final boolean brake = true;
  private static final double reduction = (1.0);

  public BackpackIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }
}
