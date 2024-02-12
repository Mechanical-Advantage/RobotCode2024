package org.littletonrobotics.frc2024.subsystems.rollers.intake;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSubsystem;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

public class Intake extends GenericRollerSubsystem<Intake.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Intake/FloorIntakingVoltage", 8.0)),
    SHOOTING(new LoggedTunableNumber("Intake/Shooting", 6.0)),
    EJECTING(new LoggedTunableNumber("Intake/EjectingVoltage", -8.0));

    private final DoubleSupplier voltageSupplier;
  }

  @Getter @Setter private Goal goal = Goal.IDLE;

  public Intake(IntakeIO io) {
    super("Intake", io);
  }
}
