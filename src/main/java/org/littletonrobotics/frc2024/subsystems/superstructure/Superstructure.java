// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  public enum SystemState {
    // PREPARE_SHOOT,
    AIM,
    // PREPARE_INTAKE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    // REVERSE_INTAKE,
    IDLE
  }

  public enum GamepieceState {
    NO_GAMEPIECE,
    HOLDING_SHOOTER,
    HOLDING_BACKPACK
  }

  @AutoLogOutput @Getter private SystemState currentState = SystemState.IDLE;
  @AutoLogOutput @Getter private SystemState goalState = SystemState.IDLE;
  @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NO_GAMEPIECE;

  private final Arm arm;

  public Superstructure(Arm arm) {
    this.arm = arm;
    setDefaultCommand(runOnce(() -> goalState = SystemState.IDLE).withName("SuperstructureIdle"));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      goalState = SystemState.IDLE;
    }

    switch (goalState) {
      case AIM -> currentState = SystemState.AIM;
      case FLOOR_INTAKE -> currentState = SystemState.FLOOR_INTAKE;
      case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
      case IDLE -> currentState = SystemState.IDLE;
    }

    switch (currentState) {
      case AIM -> {
        arm.setGoal(Arm.Goal.AIM);
      }
      case FLOOR_INTAKE -> {
        arm.setGoal(Arm.Goal.FLOOR_INTAKE);
      }
      case STATION_INTAKE -> {
        arm.setGoal(Arm.Goal.STATION_INTAKE);
      }
      case IDLE -> {
        arm.setGoal(Arm.Goal.STOW);
      }
    }

    arm.periodic();
  }

  public boolean atArmSetpoint() {
    return arm.atSetpoint();
  }

  public Command aimCommand() {
    return startEnd(() -> goalState = SystemState.AIM, () -> goalState = SystemState.IDLE)
        .withName("SuperstructureAim");
  }

  public Command floorIntakeCommand() {
    return startEnd(() -> goalState = SystemState.FLOOR_INTAKE, () -> goalState = SystemState.IDLE)
        .withName("SuperstructureFloorIntake");
  }

  public Command stationIntakeCommand() {
    return startEnd(
            () -> goalState = SystemState.STATION_INTAKE, () -> goalState = SystemState.IDLE)
        .withName("SuperstructureStationIntake");
  }
}
