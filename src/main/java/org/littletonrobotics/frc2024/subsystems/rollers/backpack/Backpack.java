// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystem;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Setter
@Getter
public class Backpack extends GenericRollerSystem<Backpack.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLING(() -> 0),
    AMP_SCORING(new LoggedTunableNumber("Backpack/AmpScoringVoltage", 12.0)),
    TRAP_SCORING(new LoggedTunableNumber("Backpack/TrapScoringVoltage", 2.0)),
    EJECTING(new LoggedTunableNumber("Backpack/EjectingVoltage", -12.0)),
    FORWARD(new LoggedTunableNumber("Backpack/DiagnoseForward", 12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.IDLING;

  public Backpack(BackpackIO io) {
    super("Backpack", io);
  }
}
