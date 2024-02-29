// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.feeder;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystem;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Setter
@Getter
public class Feeder extends GenericRollerSystem<Feeder.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLING(() -> 0.0),
    SHUFFLING(new LoggedTunableNumber("Feeder/ShufflingVoltage", 0.0)),
    FLOOR_INTAKING(new LoggedTunableNumber("Feeder/FloorIntakingVoltage", 10.0)),
    SHOOTING(new LoggedTunableNumber("Feeder/Shooting", 8.0)),
    EJECTING(new LoggedTunableNumber("Feeder/EjectingVoltage", -6.0));

    private final DoubleSupplier voltageSupplier;
  }

  private Feeder.Goal goal = Feeder.Goal.IDLING;

  public Feeder(FeederIO io) {
    super("Feeder", io);
  }
}
