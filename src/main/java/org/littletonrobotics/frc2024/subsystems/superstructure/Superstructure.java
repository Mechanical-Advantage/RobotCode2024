// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class Superstructure extends SubsystemBase {

  public enum SystemState {
    PREPARE_SHOOT,
    SHOOT,
    PREPARE_INTAKE,
    INTAKE,
    STATION_INTAKE,
    REVERSE_INTAKE,
    IDLE
  }

  public enum GamepieceState {
    NO_GAMEPIECE,

    HOLDING_SHOOTER,

    HOLDING_BACKPACK
  }

  @Getter private SystemState currentState = SystemState.IDLE;
  @Getter @Setter private SystemState goalState = SystemState.IDLE;

  @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NO_GAMEPIECE;

  private final Arm arm;
  private final Flywheels flywheels;

  private final Timer followThroughTimer = new Timer();

  @Override
  public void periodic() {
    switch (goalState) {
      case IDLE -> currentState = SystemState.IDLE;
      case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
      case INTAKE -> currentState = SystemState.INTAKE;
      case PREPARE_SHOOT -> currentState = SystemState.PREPARE_SHOOT;
      case SHOOT -> currentState = SystemState.SHOOT;
    }

    Logger.recordOutput("Superstructure/GoalState", goalState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
  }

  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean atShootingSetpoint() {
    return flywheels.atSetpoint();
  }
}
