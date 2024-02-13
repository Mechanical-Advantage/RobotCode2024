package org.littletonrobotics.frc2024.subsystems.rollers.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSim;

public class IntakeIOSim extends GenericRollerSystemIOSim implements IntakeIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
  private static final double reduction = (18.0 / 12.0);
  private static final double moi = 0.001;

  public IntakeIOSim() {
    super(motorModel, reduction, moi);
  }
}
