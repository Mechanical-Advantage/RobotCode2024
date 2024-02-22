// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevator;

@Getter
@Setter
public class BackpackActuator extends GenericSlamElevator<BackpackActuator.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements SlamElevatorGoal {
    RETRACT(-1, true),
    EXTEND(1, true);

    private final int direction;
    private final boolean stopAtGoal;
  }

  private Goal goal = Goal.RETRACT;

  public BackpackActuator(BackpackActuatorIO io) {
    super("BackpackActuator", io, 40.0, 0.2, 0.05);
  }
}
