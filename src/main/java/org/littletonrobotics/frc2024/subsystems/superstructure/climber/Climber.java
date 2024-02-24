// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.climber;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevator;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Getter
@Setter
public class Climber extends GenericSlamElevator<Climber.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements SlamElevatorGoal {
    IDLE(new LoggedTunableNumber("Climber/IdleCurrent", 0.0), true, SlamElevatorState.IDLING),
    RETRACT(new LoggedTunableNumber("Climber/Retract", -40.0), false, SlamElevatorState.RETRACTED),
    EXTEND(
        new LoggedTunableNumber("Climber/ExtendingCurrent", 20.0),
        true,
        SlamElevatorState.EXTENDED);

    private final DoubleSupplier slammingCurrent;
    private final boolean stopAtGoal;
    private final SlamElevatorState state;
  }

  private Goal goal = Goal.IDLE;

  public Climber(ClimberIO io) {
    super("Climber", io, 0.4, 0.1);
  }
}
