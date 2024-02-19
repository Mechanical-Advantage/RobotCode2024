// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.flywheels;

import static org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.frc2024.util.LinearProfile;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());
  private static final LoggedTunableNumber shootingLeftRpm =
      new LoggedTunableNumber("Flywheels/ShootingLeftRpm", 6000.0);
  private static final LoggedTunableNumber shootingRightRpm =
      new LoggedTunableNumber("Flywheels/ShootingRightRpm", 4000.0);
  private static final LoggedTunableNumber idleLeftRpm =
      new LoggedTunableNumber("Flywheels/IdleLeftRpm", 1500.0);
  private static final LoggedTunableNumber idleRightRpm =
      new LoggedTunableNumber("Flywheels/IdleRightRpm", 1000.0);
  private static final LoggedTunableNumber intakingRpm =
      new LoggedTunableNumber("Flywheels/IntakingRpm", -2000.0);
  private static final LoggedTunableNumber ejectingRpm =
      new LoggedTunableNumber("Flywheels/EjectingRpm", 1000.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;

  // Disconnected alerts
  private final Alert leftDisconnectedAlert =
      new Alert("Left Flywheel Disconnected!", Alert.AlertType.WARNING);
  private final Alert rightDisconnectedAlert =
      new Alert("Left Flywheel Disconnected!", Alert.AlertType.WARNING);

  @RequiredArgsConstructor
  public enum Goal {
    STOP(() -> 0, () -> 0),
    IDLE(idleLeftRpm, idleRightRpm),
    SHOOT(shootingLeftRpm, shootingRightRpm),
    INTAKE(intakingRpm, intakingRpm),
    EJECT(ejectingRpm, ejectingRpm),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    private double getLeftGoal() {
      return leftGoal.getAsDouble();
    }

    private double getRightGoal() {
      return rightGoal.getAsDouble();
    }
  }

  @Getter private Goal goal = Goal.IDLE;

  public Flywheels(FlywheelsIO io) {
    this.io = io;

    leftProfile = new LinearProfile(maxAcceleration.get(), Constants.loopPeriodSecs);
    rightProfile = new LinearProfile(maxAcceleration.get(), Constants.loopPeriodSecs);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // Set alerts
    leftDisconnectedAlert.set(!inputs.leftConnected);
    rightDisconnectedAlert.set(!inputs.rightConnected);

    // Check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> io.setFF(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          leftProfile.setMaxAcceleration(maxAcceleration.get());
          rightProfile.setMaxAcceleration(maxAcceleration.get());
        },
        maxAcceleration);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setGoal(Goal.STOP);
    }

    // Check if profile needs to be reset
    if (!closedLoop && wasClosedLoop) {
      leftProfile.reset();
      rightProfile.reset();
      wasClosedLoop = false;
    }

    // Run to setpoint
    if (closedLoop) {
      // Update goals
      leftProfile.setGoal(goal.getLeftGoal());
      rightProfile.setGoal(goal.getRightGoal());
      io.runVelocity(leftProfile.calculateSetpoint(), rightProfile.calculateSetpoint());
    }

    Logger.recordOutput("Flywheels/Goal", goal);
    Logger.recordOutput("Flywheels/SetpointLeftRpm", leftProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/SetpointRightRpm", rightProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/GoalLeftRpm", goal.getLeftGoal());
    Logger.recordOutput("Flywheels/GoalRightRpm", goal.getRightGoal());
  }

  /** Set the current goal of the flywheel */
  private void setGoal(Goal goal) {
    if (goal == Goal.CHARACTERIZING || goal == Goal.STOP) {
      wasClosedLoop = closedLoop;
      closedLoop = false;
      return; // Don't set a goal
    }
    // If not already controlling to requested goal
    // set closed loop false
    closedLoop = this.goal == goal;
    // Enable close loop
    if (!closedLoop) {
      leftProfile.setGoal(goal.getLeftGoal(), inputs.leftVelocityRpm);
      rightProfile.setGoal(goal.getRightGoal(), inputs.rightVelocityRpm);
      closedLoop = true;
    }
    this.goal = goal;
  }

  /** Run characterization with input in either current or amps */
  public void runCharacterization(double input) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationLeft(input);
    io.runCharacterizationRight(input);
  }

  /** Get characterization velocity */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  /** Get if velocity profile has ended */
  @AutoLogOutput(key = "Flywheels/AtGoal")
  public boolean atGoal() {
    return leftProfile.getCurrentSetpoint() == goal.getLeftGoal()
        && rightProfile.getCurrentSetpoint() == goal.getRightGoal();
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE)).withName("FlywheelsShoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE))
        .withName("FlywheelsIntake");
  }
}
