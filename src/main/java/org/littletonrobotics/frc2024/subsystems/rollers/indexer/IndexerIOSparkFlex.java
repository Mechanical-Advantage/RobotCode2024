package org.littletonrobotics.frc2024.subsystems.rollers.indexer;

import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSparkFlex;

public class IndexerIOSparkFlex extends GenericRollerSystemIOSparkFlex implements IndexerIO {
  private static final double reduction = (18.0 / 12.0);
  private static final int id = 6;
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = false;

  public IndexerIOSparkFlex() {
    super(id, currentLimitAmps, inverted, true, reduction);
  }
}
