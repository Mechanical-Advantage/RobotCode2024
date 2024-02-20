// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSim;

public class BackpackIOSim extends GenericRollerSystemIOSim implements BackpackIO {
  private static final DCMotor motorModel = DCMotor.getNeoVortex(1);
  private static final double reduction = (1.0 / 1.0);
  private static final double moi = 0.001;

  public BackpackIOSim() {
    super(motorModel, reduction, moi);
  }
}
