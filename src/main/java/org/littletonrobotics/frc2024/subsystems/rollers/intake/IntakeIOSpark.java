// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.intake;

import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSpark;

public class IntakeIOSpark extends GenericRollerSystemIOSpark implements IntakeIO {

  private static final int id = 2;
  private static final int currentLimitAmps = 40;
  private static final boolean invert = true;
  private static final boolean brake = false;
  private static final double reduction = 18.0 / 12.0;

  public IntakeIOSpark() {
    super(id, currentLimitAmps, invert, brake, reduction, false);
  }
}
