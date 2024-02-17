package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import org.littletonrobotics.frc2024.util.drivers.rollers.GenericRollerSystemIOSparkFlex;

public class BackpackIOSparkFlex extends GenericRollerSystemIOSparkFlex implements BackpackIO {
    private static final int id = 0;
    private static final int currentLimitAmps = 40;
    private static final boolean invert = false;
    private static final boolean brake = true;
    private static final double reduction = (1.0 / 1.0);

    public BackpackIOSparkFlex() {
        super(id, currentLimitAmps, invert, brake, reduction);
    }
}
