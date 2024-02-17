package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.frc2024.util.drivers.rollers.GenericRollerSystem;

import java.util.function.DoubleSupplier;

@Setter
@Getter
public class Backpack extends GenericRollerSystem<Backpack.Goal> {
    @RequiredArgsConstructor
    @Getter
    public enum Goal implements VoltageGoal {
        IDLING(() -> 0),
        AMP_SCORING(new LoggedTunableNumber("Backpack/AmpScoringVoltage"));

        private final DoubleSupplier voltageSupplier;
    }

    private Goal goal = Goal.IDLING;

    public Backpack(BackpackIO io) {
        super("Backpack", io);
    }
}
