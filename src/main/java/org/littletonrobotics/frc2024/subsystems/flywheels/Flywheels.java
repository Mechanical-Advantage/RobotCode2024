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
  private static final LoggedTunableNumber shootingLeftRPM =
      new LoggedTunableNumber("Flywheels/ShootingLeftRPM", 6000.0);
  private static final LoggedTunableNumber shootingRightRPM =
      new LoggedTunableNumber("Flywheels/ShootingRightRPM", 4000.0);
  private static final LoggedTunableNumber idleLeftRPM =
      new LoggedTunableNumber("Flywheels/IdleLeftRPM", 1500.0);
  private static final LoggedTunableNumber idleRightRPM =
      new LoggedTunableNumber("Flywheels/IdleRightRPM", 1000.0);
  private static final LoggedTunableNumber intakingLeftRPM =
      new LoggedTunableNumber("Flywheels/IntakingLeftRPM", -2000.0);
  private static final LoggedTunableNumber intakingRightRPM =
      new LoggedTunableNumber("Flywheels/IntakingRightRPM", -2000.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;

  @RequiredArgsConstructor
  public enum Goal {
    STOP(() -> 0.0, () -> 0.0),
    IDLE(idleLeftRPM, idleRightRPM),
    SHOOTING(shootingLeftRPM, shootingRightRPM),
    INTAKING(intakingLeftRPM, intakingRightRPM),
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

    // check controllers
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

  public void runCharacterizationVolts(double volts) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationLeftVolts(volts);
    io.runCharacterizationRightVolts(volts);
  }

  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  @AutoLogOutput(key = "Flywheels/AtGoal")
  public boolean atGoal() {
    return leftProfile.getCurrentSetpoint() == goal.getLeftGoal()
        && rightProfile.getCurrentSetpoint() == goal.getRightGoal();
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOTING), () -> setGoal(Goal.IDLE))
        .withName("FlywheelsShoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKING), () -> setGoal(Goal.IDLE))
        .withName("FlywheelsIntake");
  }
}
