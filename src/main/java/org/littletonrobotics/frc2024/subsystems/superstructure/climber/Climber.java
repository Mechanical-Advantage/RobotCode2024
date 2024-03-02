// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.climber;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevator;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevator.SlamElevatorGoal;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

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
    IDLE(new LoggedTunableNumber("Climber/IdleCurrent", -8.0), true, SlamElevatorState.RETRACTING),
    RETRACT(
        new LoggedTunableNumber("Climber/RetractingCurrent", -10.0), // 10.0
        false,
        SlamElevatorState.RETRACTING),
    EXTEND(
        new LoggedTunableNumber("Climber/ExtendingCurrent", 8.0),
        true,
        SlamElevatorState.EXTENDING);

    private final DoubleSupplier slammingCurrent;
    private final boolean stopAtGoal;
    private final SlamElevatorState state;
  }

  private Goal goal = Goal.IDLE;
  private boolean hasHome = false;
  private double topHome = 0.0;

  private boolean hasCheckedCancel = false;
  @Getter private boolean requestCancelClimb = false;

  public Climber(ClimberIO io) {
    super("Climber", io, 0.4, 0.1);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (getGoal().getState() != SlamElevatorState.RETRACTING) {
      hasCheckedCancel = false;
      requestCancelClimb = false;
    }

    if (extended()) {
      hasHome = true;
      topHome = inputs.positionRads;
    }

    if (DriverStation.isDisabled()) {
      hasHome = false;
      hasCheckedCancel = false;
    }

    // Stop climbing if we are applying too much current
    // Superstructure cancel climb
    if (hasHome
        && Math.abs((inputs.positionRads - topHome) * drumRadiusInches)
            >= cancelClimbCheckInches.get()
        && !hasCheckedCancel) {
      if (Math.abs(inputs.torqueCurrentAmps) <= cancelClimbMaxTorqueCurrent.get()) {
        //requestCancelClimb = true;
      }
      hasCheckedCancel = true;
    }

    Logger.recordOutput("Superstructure/Climber/HasHome", hasHome);
    Logger.recordOutput("Superstructure/Climber/HasCheckedCancel", hasCheckedCancel);
    Logger.recordOutput("Superstructure/Climber/RequestedCancelClimb", requestCancelClimb);
  }
}
