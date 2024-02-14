// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class Superstructure extends SubsystemBase {

  public enum SystemState {
    STOWING,
    AIMING,
    INTAKING,
    STATION_INTAKING,
    AMP,
    PREPARE_CLIMB,
    CLIMB,
    TRAP
  }

  @Getter private SystemState currentState = SystemState.STOWING;
  @Getter @Setter private SystemState goal = SystemState.STOWING;

  private final Arm arm;

  @Override
  public void periodic() {
    currentState = goal; // Will change soon

    switch (currentState) {
      case STOWING -> arm.setGoal(Arm.Goal.STOW);
      case AIMING -> arm.setGoal(Arm.Goal.AIM);
      case INTAKING -> arm.setGoal(Arm.Goal.FLOOR_INTAKE);
      case STATION_INTAKING -> arm.setGoal(Arm.Goal.STATION_INTAKE);
      default -> {} // DO NOTHING ELSE
    }

    arm.periodic();

    Logger.recordOutput("Superstructure/GoalState", goal);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
  }

  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean atShootingSetpoint() {
    return currentState == SystemState.AIMING && arm.atSetpoint();
  }

  @AutoLogOutput(key = "Superstructure/AtGoalState")
  public boolean atGoal() {
    return arm.atSetpoint();
  }

  public Command stow() {
    return runOnce(() -> setGoal(SystemState.STOWING)).withName("Superstructure Stow");
  }

  public Command aim() {
    return startEnd(() -> setGoal(SystemState.AIMING), () -> setGoal(SystemState.STOWING))
        .withName("Superstructure Aim");
  }

  public Command intake() {
    return startEnd(() -> setGoal(SystemState.INTAKING), () -> setGoal(SystemState.STOWING))
        .withName("Superstructure Intake");
  }

  public Command stationIntake() {
    return startEnd(() -> setGoal(SystemState.STATION_INTAKING), () -> setGoal(SystemState.STOWING))
        .withName("Superstructure Station Intake");
  }
}
