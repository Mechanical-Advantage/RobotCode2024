// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevator;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Getter
@Setter
public class BackpackActuator extends GenericSlamElevator<BackpackActuator.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements SlamElevatorGoal {
    RETRACT(
        new LoggedTunableNumber("BackpackActuator/RetractingCurrent", -30.0),
        true,
        SlamElevatorState.RETRACTED),
    EXTEND(
        new LoggedTunableNumber("BackpackActuator/ExtendingCurrent", 30.0),
        true,
        SlamElevatorState.EXTENDED);

    private final DoubleSupplier slammingCurrent;
    private final boolean stopAtGoal;
    private final SlamElevatorState state;
  }

  private Goal goal = Goal.RETRACT;

  public BackpackActuator(BackpackActuatorIO io) {
    super("BackpackActuator", io, 0.2, 0.05);
  }
}
