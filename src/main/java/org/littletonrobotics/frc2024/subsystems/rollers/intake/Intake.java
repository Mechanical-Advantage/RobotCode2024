// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.intake;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.drivers.rollers.GenericRollerSystem;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Setter
@Getter
public class Intake extends GenericRollerSystem<Intake.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLING(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Intake/FloorIntakingVoltage", 8.0)),
    EJECTING(new LoggedTunableNumber("Intake/EjectingVoltage", -8.0));

    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.IDLING;

  public Intake(IntakeIO io) {
    super("Intake", io);
  }
}
