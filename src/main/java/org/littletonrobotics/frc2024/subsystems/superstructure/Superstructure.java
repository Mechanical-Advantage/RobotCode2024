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

  @Getter private Goal currentGoal = Goal.STOW;
  @Getter private Goal desiredGoal = Goal.STOW;

  private final Arm arm;

  public Superstructure(Arm arm) {
    this.arm = arm;

    setDefaultCommand(setGoalCommand(Goal.STOW));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      desiredGoal = Goal.STOW;
      arm.stop();
    }

    currentGoal = desiredGoal; // Will change soon

    switch (currentGoal) {
      case STOW -> arm.setGoal(Arm.Goal.STOW);
      case AIM -> arm.setGoal(Arm.Goal.AIM);
      case INTAKE -> arm.setGoal(Arm.Goal.FLOOR_INTAKE);
      case STATION_INTAKE -> arm.setGoal(Arm.Goal.STATION_INTAKE);
      case DIAGNOSTIC_ARM -> arm.setGoal(Arm.Goal.CUSTOM);
      case AMP -> arm.setGoal(Arm.Goal.AMP);
      case SUBWOOFER -> arm.setGoal(Arm.Goal.SUBWOOFER);
      case PODIUM -> arm.setGoal(Arm.Goal.PODIUM);
      default -> {} // DO NOTHING ELSE
    }

    arm.periodic();

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
