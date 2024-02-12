package org.littletonrobotics.frc2024.subsystems.rollers.feeder;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSubsystem;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

public class Feeder extends GenericRollerSubsystem<Feeder.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements GenericRollerSubsystem.VoltageGoal {
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Feeder/FloorIntakingVoltage", 8.0)),
    BACKSTOPPING(new LoggedTunableNumber("Feeder/BackstoppingVoltage", -4.0)),
    SHOOTING(new LoggedTunableNumber("Feeder/Shooting", 8.0)),
    EJECTING(new LoggedTunableNumber("Feeder/EjectingVoltage", -6.0));

    private final DoubleSupplier voltageSupplier;
  }

  @Getter @Setter private Feeder.Goal goal = Feeder.Goal.IDLE;

  public Feeder(FeederIO io) {
    super("Feeder", io);
  }
}
