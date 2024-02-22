// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.climber;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevator;

@Getter
@Setter
public class Climber extends GenericSlamElevator<Climber.Goal> {

  @RequiredArgsConstructor
  @Getter
  public enum Goal implements SlamElevatorGoal {
    RETRACT_AND_IDLE(-1, true),
    RETRACT(-1, false),
    EXTEND(1, true);

    private final int direction;
    private final boolean stopAtGoal;
  }

  private Goal goal = Goal.RETRACT_AND_IDLE;

  public Climber(ClimberIO io) {
    super("Climber", io, 40.0, 0.4, 0.1);
  }
}
