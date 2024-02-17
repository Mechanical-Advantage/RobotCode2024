package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.frc2024.util.drivers.rollers.GenericRollerSystemIOSim;

public class BackpackIOSim extends GenericRollerSystemIOSim implements BackpackIO {
    private static final DCMotor motorModel = DCMotor.getNeoVortex(1);
    private static final double reduction = (1.0 / 1.0);
    private static final double moi = 0.001;

    public BackpackIOSim() {
        super(motorModel, reduction, moi);
    }
}
