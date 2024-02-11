package org.littletonrobotics.frc2024.subsystems.rollers.indexer;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSubsystem;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Setter
@Getter
public class Indexer extends GenericRollerSubsystem<Indexer.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Indexer/FloorIntakingVoltage", 2.0)),
    STATION_INTAKING(new LoggedTunableNumber("Indexer/StationIntakingVoltage", -2.0)),
    SHOOTING(new LoggedTunableNumber("Indexer/ShootingVoltage", 12.0)),
    EJECTING(new LoggedTunableNumber("Indexer/EjectingVoltage", -8.0));

    private final DoubleSupplier voltageSupplier;
  }

  @Getter @Setter private Indexer.Goal goal = Indexer.Goal.IDLE;

  public Indexer(IndexerIO io) {
    super("Indexer", io);
  }
}
