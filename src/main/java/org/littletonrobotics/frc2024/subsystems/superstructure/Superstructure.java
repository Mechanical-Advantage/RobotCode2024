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
    PREPARE_CLIMB,
    CLIMB,
    TRAP,
    DIAGNOSTIC_ARM
  }

  @Getter private Goal currentGoal = Goal.STOW;
  @Getter private Goal desiredGoal = Goal.STOW;

  private final Arm arm;
  private final Climber climber;

  public Superstructure(Arm arm, Climber climber) {
    this.arm = arm;
    this.climber = climber;

    setDefaultCommand(setGoalCommand(Goal.STOW));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      desiredGoal = Goal.STOW;
      arm.stop();
    }

    // Handle transitions
    if (desiredGoal != Goal.PREPARE_CLIMB && !climber.retracted()) {
      // Wait for retracted before setting
      climber.setGoal(GenericSlamElevator.Goal.RETRACT);
    } else if (desiredGoal == Goal.CLIMB && !climber.extended()) {
      // Finish preparing climb before climbing
      currentGoal = Goal.PREPARE_CLIMB;
    } else {
      currentGoal = desiredGoal;
    }

    switch (currentGoal) {
      case STOW -> {
        arm.setGoal(Arm.Goal.STOW);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
      case AIM -> {
        arm.setGoal(Arm.Goal.AIM);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
      case INTAKE -> {
        arm.setGoal(Arm.Goal.FLOOR_INTAKE);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
      case STATION_INTAKE -> {
        arm.setGoal(Arm.Goal.STATION_INTAKE);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
      case AMP -> {
        arm.setGoal(Arm.Goal.AMP);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
      case SUBWOOFER -> {
        arm.setGoal(Arm.Goal.SUBWOOFER);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
      case PREPARE_CLIMB -> {
        arm.setGoal(Arm.Goal.CLIMB);
        climber.setGoal(GenericSlamElevator.Goal.EXTEND);
      }
      case CLIMB -> {
        arm.setGoal(Arm.Goal.CLIMB);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
      case DIAGNOSTIC_ARM -> {
        arm.setGoal(Arm.Goal.CUSTOM);
        climber.setGoal(GenericSlamElevator.Goal.RETRACT);
      }
    }

    arm.periodic();
    climber.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> desiredGoal = goal, () -> desiredGoal = Goal.STOW)
        .withName("Superstructure " + goal);
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && arm.atGoal() & climber.atGoal();
  }

  public Command runArmCharacterization() {
    return arm.getStaticCurrent().finallyDo(() -> desiredGoal = Goal.STOW);
  }
}
