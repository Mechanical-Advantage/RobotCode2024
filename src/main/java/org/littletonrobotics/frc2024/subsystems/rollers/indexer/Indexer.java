// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.indexer;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystem;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystem.VoltageGoal;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Setter
@Getter
public class Indexer extends GenericRollerSystem<Indexer.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLING(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Indexer/FloorIntakingVoltage", 6.0)),
    STATION_INTAKING(new LoggedTunableNumber("Indexer/StationIntakingVoltage", -6.0)),
    SHOOTING(new LoggedTunableNumber("Indexer/ShootingVoltage", 12.0)),
    EJECTING(new LoggedTunableNumber("Indexer/EjectingVoltage", -8.0)),
    DIAGNOSING(new LoggedTunableNumber("Backpack/DiagnosingVoltage", 0.0));

    private final DoubleSupplier voltageSupplier;
  }

  private Indexer.Goal goal = Indexer.Goal.IDLING;

  public Indexer(IndexerIO io) {
    super("Indexer", io);
  }
}
