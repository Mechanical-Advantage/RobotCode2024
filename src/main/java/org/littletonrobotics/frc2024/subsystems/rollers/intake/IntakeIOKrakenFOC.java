// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.intake;

import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOKrakenFOC;

public class IntakeIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements IntakeIO {
  private static final int id;
  private static final String bus = "*";
  private static final int currentLimitAmps = 40;
  private static final boolean invert;
  private static final boolean brake = false;
  private static final double reduction = 18.0 / 12.0;

  static {
    if (Constants.getRobot() == Constants.RobotType.COMPBOT) {
      id = 9;
      invert = false;
    } else {
      id = 2;
      invert = false;
    }
  }

  public IntakeIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }
}
