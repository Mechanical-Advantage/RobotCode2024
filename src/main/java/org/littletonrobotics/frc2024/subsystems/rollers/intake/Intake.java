// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystem;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystem.VoltageGoal;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Setter
@Getter
public class Intake extends GenericRollerSystem<Intake.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLING(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Intake/FloorIntakingVoltage", 10.0)),
    EJECTING(new LoggedTunableNumber("Intake/EjectingVoltage", -8.0));

    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public Intake(IntakeIO io) {
    super("Intake", io);
  }

  public boolean isTouchingNote() {
    return goal == Goal.FLOOR_INTAKING
        && stateTimer.hasElapsed(0.25)
        && currentDebouncer.calculate(inputs.torqueCurrentAmps > 45.0);
  }
}
