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
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator.BackpackActuator;
import org.littletonrobotics.frc2024.subsystems.superstructure.climber.Climber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    STOW,
    AIM,
    INTAKE,
    STATION_INTAKE,
    AMP,
    SUBWOOFER,
    PODIUM,
    PREPARE_CLIMB,
    CLIMB,
    TRAP,
    DIAGNOSTIC_ARM
  }

  private Goal lastGoal = Goal.STOW;
  @Getter private Goal currentGoal = Goal.STOW;
  @Getter private Goal desiredGoal = Goal.STOW;

  private final Arm arm;
  private final Climber climber;
  private final BackpackActuator backpackActuator;

  public Superstructure(Arm arm, Climber climber, BackpackActuator backpackActuator) {
    this.arm = arm;
    this.climber = climber;
    this.backpackActuator = backpackActuator;

    setDefaultCommand(setGoalCommand(Goal.STOW));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      desiredGoal = Goal.STOW;
      arm.stop();
    }

    currentGoal = desiredGoal;

    switch (currentGoal) {
      case STOW -> {
        arm.setGoal(Arm.Goal.STOW);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case AIM -> {
        arm.setGoal(Arm.Goal.AIM);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case INTAKE -> {
        arm.setGoal(Arm.Goal.FLOOR_INTAKE);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case STATION_INTAKE -> {
        arm.setGoal(Arm.Goal.STATION_INTAKE);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case AMP -> {
        arm.setGoal(Arm.Goal.AMP);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case SUBWOOFER -> {
        arm.setGoal(Arm.Goal.SUBWOOFER);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case PREPARE_CLIMB -> {
        arm.setGoal(Arm.Goal.PREPARE_CLIMB);
        climber.setGoal(Climber.Goal.EXTEND);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case CLIMB -> {
        arm.setGoal(Arm.Goal.CLIMB);
        if (climber.isRequestCancelClimb()) {
          desiredGoal = Goal.PREPARE_CLIMB;
        } else {
          climber.setGoal(Climber.Goal.RETRACT);
        }
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case TRAP -> {
        arm.setGoal(Arm.Goal.CLIMB);
        if (climber.isRequestCancelClimb()) {
          desiredGoal = Goal.PREPARE_CLIMB;
        } else {
          climber.setGoal(Climber.Goal.RETRACT);
        }
        backpackActuator.setGoal(BackpackActuator.Goal.EXTEND);
      }
      case DIAGNOSTIC_ARM -> {
        arm.setGoal(Arm.Goal.CUSTOM);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
    }
    lastGoal = currentGoal;

    arm.periodic();
    climber.periodic();
    backpackActuator.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> desiredGoal = goal, () -> desiredGoal = Goal.STOW)
        .withName("Superstructure " + goal);
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal
        && arm.atGoal() & climber.atGoal()
        && backpackActuator.atGoal();
  }

  public void runArmCharacterization(double input) {
    arm.runCharacterization(input);
  }

  public double getArmCharacterizationVelocity() {
    return arm.getCharacterizationVelocity();
  }

  public void endArmCharacterization() {
    arm.endCharacterization();
  }
}
