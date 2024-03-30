// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSpark;

public class BackpackIOSparkFlex extends GenericRollerSystemIOSpark implements BackpackIO {
  private static final int id = 6;
  private static final int currentLimitAmps = 40;
  private static final boolean invert = true;
  private static final boolean brake = true;
  private static final double reduction = (1.0);

  public BackpackIOSparkFlex() {
    super(id, currentLimitAmps, invert, brake, reduction, true);
  }
}
