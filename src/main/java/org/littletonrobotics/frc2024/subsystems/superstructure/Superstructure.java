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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    STOW,
    AIM,
    INTAKE,
    STATION_INTAKE,
    AMP,
    PREPARE_CLIMB,
    CLIMB,
    TRAP
  }

  @Getter private Goal currentGoal = Goal.STOW;
  @Getter private Goal desiredGoal = Goal.STOW;

  private final Arm arm;

  public Superstructure(Arm arm) {
    this.arm = arm;

    setDefaultCommand(runOnce(this::stow).withName("Superstructure Stowing"));
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
      case AMP -> arm.setGoal(Arm.Goal.AMP);
      default -> {} // DO NOTHING ELSE
    }

    arm.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && arm.atGoal();
  }

  public void stow() {
    desiredGoal = Goal.STOW;
  }

  public Command aim() {
    return startEnd(() -> desiredGoal = Goal.AIM, this::stow).withName("Superstructure Aiming");
  }

  public Command intake() {
    return startEnd(() -> desiredGoal = Goal.INTAKE, this::stow)
        .withName("Superstructure Intaking");
  }

  public Command amp() {
    return startEnd(() -> desiredGoal = Goal.AMP, this::stow).withName("Superstructure AMPing");
  }

  public Command stationIntake() {
    return startEnd(() -> desiredGoal = Goal.STATION_INTAKE, this::stow)
        .withName("Superstructure Station Intaking");
  }
}
