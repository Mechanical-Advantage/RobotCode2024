package org.littletonrobotics.frc2024.subsystems.rollers.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSim;

public class IndexerIOSim extends GenericRollerSystemIOSim implements IndexerIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
  private static final double reduction = (1.0 / 1.0);
  private static final double moi = 0.001;

  public IndexerIOSim() {
    super(motorModel, reduction, moi);
  }
}
