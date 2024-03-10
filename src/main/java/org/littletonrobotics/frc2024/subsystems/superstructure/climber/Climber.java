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
  private static final LoggedTunableNumber cancelClimbCheckInches =
      new LoggedTunableNumber("Climber/CancelClimbInches", 14.0);
  private static final LoggedTunableNumber cancelClimbMaxTorqueCurrent =
      new LoggedTunableNumber("Climber/CancelClimbTorqueCurrent", 15.0);
  private static final double drumRadiusInches = 1.275;

  @RequiredArgsConstructor
  @Getter
  public enum Goal implements SlamElevatorGoal {
    STOP(new LoggedTunableNumber("Climber/StopCurrent", 0.0), false, SlamElevatorState.IDLING),
    IDLE(new LoggedTunableNumber("Climber/IdleCurrent", -12.0), true, SlamElevatorState.RETRACTING),
    RETRACT(
        new LoggedTunableNumber("Climber/RetractingCurrent", -30.0),
        false,
        SlamElevatorState.RETRACTING),
    EXTEND(
        new LoggedTunableNumber("Climber/ExtendingCurrent", 12.0),
        true,
        SlamElevatorState.EXTENDING);

    private final DoubleSupplier slammingCurrent;
    private final boolean stopAtGoal;
    private final SlamElevatorState state;
  }

  private Goal goal = Goal.IDLE;

  public Climber(ClimberIO io) {
    super("Climber", io, 0.4, 0.1);
  }
}
