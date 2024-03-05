// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    BACKPACK_OUT_UNJAM,
    AIM,
    INTAKE,
    UNJAM_INTAKE,
    STATION_INTAKE,
    AMP,
    SUBWOOFER,
    PODIUM,
    RESET_CLIMB,
    PREPARE_CLIMB,
    CANCEL_PREPARE_CLIMB,
    CLIMB,
    CANCEL_CLIMB,
    TRAP,
    RESET,
    DIAGNOSTIC_ARM
  }

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
      setDefaultCommand(setGoalCommand(Goal.STOW));
      arm.stop();
    }

    // Retract climber
    if (!climber.retracted()
        && desiredGoal != Goal.PREPARE_CLIMB
        && desiredGoal != Goal.CLIMB
        && desiredGoal != Goal.TRAP
        && !DriverStation.isAutonomousEnabled()) {
      currentGoal = Goal.RESET_CLIMB;
    } else {
      currentGoal = desiredGoal;
    }

    if (desiredGoal == Goal.CANCEL_CLIMB || desiredGoal == Goal.CANCEL_PREPARE_CLIMB) {
      currentGoal = desiredGoal;
    }

    switch (currentGoal) {
      case STOW -> {
        arm.setGoal(Arm.Goal.STOW);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case BACKPACK_OUT_UNJAM -> {
        arm.setGoal(Arm.Goal.UNJAM_INTAKE);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.EXTEND);
      }
      case UNJAM_INTAKE -> {
        arm.setGoal(Arm.Goal.UNJAM_INTAKE);
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
      case RESET_CLIMB -> {
        arm.setGoal(Arm.Goal.STOP);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case PREPARE_CLIMB -> {
        arm.setGoal(Arm.Goal.PREPARE_CLIMB);
        climber.setGoal(Climber.Goal.EXTEND);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case CANCEL_PREPARE_CLIMB -> {
        arm.setGoal(Arm.Goal.STOP);
        climber.setGoal(Climber.Goal.STOP);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case CLIMB -> {
        arm.setGoal(Arm.Goal.CLIMB);
        if (climber.isRequestCancelClimb()) {
          desiredGoal = Goal.CANCEL_CLIMB;
        } else {
          climber.setGoal(Climber.Goal.RETRACT);
        }
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
      case CANCEL_CLIMB -> {
        arm.setGoal(Arm.Goal.CLIMB);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
        climber.setGoal(Climber.Goal.STOP);
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
      case RESET -> {
        desiredGoal = Goal.STOW;
        setDefaultCommand(setGoalCommand(Goal.STOW));
      }
      case DIAGNOSTIC_ARM -> {
        arm.setGoal(Arm.Goal.CUSTOM);
        climber.setGoal(Climber.Goal.IDLE);
        backpackActuator.setGoal(BackpackActuator.Goal.RETRACT);
      }
    }

    arm.periodic();
    climber.periodic();
    backpackActuator.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  /** Set goal of superstructure */
  private void setGoal(Goal goal) {
    if (desiredGoal == goal) return;
    desiredGoal = goal;
  }

  /** Command to set goal of superstructure */
  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW))
        .withName("Superstructure " + goal);
  }

  /** Command to set goal of superstructure with additional profile constraints on arm */
  public Command setGoalWithConstraintsCommand(
      Goal goal, TrapezoidProfile.Constraints armProfileConstraints) {
    return setGoalCommand(goal)
        .beforeStarting(() -> arm.setProfileConstraints(armProfileConstraints))
        .finallyDo(() -> arm.setProfileConstraints(Arm.maxProfileConstraints.get()));
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && arm.atGoal() && climber.atGoal();
  }

  @AutoLogOutput(key = "Superstructure/AtArmGoal")
  public boolean atArmGoal() {
    return currentGoal == desiredGoal && arm.atGoal();
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
