// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.indexer;

import org.littletonrobotics.frc2024.util.drivers.rollers.GenericRollerSystemIOSparkFlex;

public class IndexerIOSparkFlex extends GenericRollerSystemIOSparkFlex implements IndexerIO {
  private static final double reduction = (18.0 / 12.0);
  private static final int id = 6;
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = false;

  public IndexerIOSparkFlex() {
    super(id, currentLimitAmps, inverted, true, reduction);
  }
}
