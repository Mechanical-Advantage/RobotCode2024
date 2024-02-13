// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.feeder;

import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOKrakenFOC;

public class FeederIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements FeederIO {
  private static final int id = 3;
  private static final String bus = "rio";
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final boolean brake = false;
  private static final double reduction = 18.0 / 12.0;

  public FeederIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }
}
