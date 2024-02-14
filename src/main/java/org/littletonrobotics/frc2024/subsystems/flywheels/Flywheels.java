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
import lombok.Setter;
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
      new LoggedTunableNumber("Superstructure/ShootingLeftRpm", 6000.0);
  private static final LoggedTunableNumber shootingRightRpm =
      new LoggedTunableNumber("Superstructure/ShootingRightRpm", 4000.0);
  private static final LoggedTunableNumber idleLeftRpm =
      new LoggedTunableNumber("Superstructure/IdleLeftRpm", 1500.0);
  private static final LoggedTunableNumber idleRightRpm =
      new LoggedTunableNumber("Superstructure/IdleRightRpm", 1000.0);
  private static final LoggedTunableNumber intakingRpm =
      new LoggedTunableNumber("Superstructure/IntakingRpm", -2000.0);
  private static final LoggedTunableNumber ejectingRpm =
      new LoggedTunableNumber("Superstructure/EjectingRpm", 2000.0);
  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Flywheels/ToleranceRpm", config.toleranceRpm());

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  @RequiredArgsConstructor
  public enum Goal {
    STOP(() -> 0.0, () -> 0.0),
    IDLING(idleLeftRpm, idleRightRpm),
    SHOOTING(shootingLeftRpm, shootingRightRpm),
    INTAKING(intakingRpm, intakingRpm),
    EJECTING(ejectingRpm, ejectingRpm),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftSetpoint;
    private final DoubleSupplier rightSetpoint;

    private double getLeftSetpoint() {
      return leftSetpoint.getAsDouble();
    }

    private double getRightSetpoint() {
      return rightSetpoint.getAsDouble();
    }
  }

  @Getter @Setter private Goal goal = Goal.IDLING;
  private double characterizationVolts = 0.0;
  private boolean characterizing = false;

  public Flywheels(FlywheelsIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;

    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> io.setFF(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);

    if (DriverStation.isDisabled()) {
      setGoal(Goal.STOP);
    }

    switch (goal) {
      case STOP -> io.stop();
      case CHARACTERIZING -> {} // Handled by runCharacterizationVolts
      default -> io.runVelocity(goal.getLeftSetpoint(), goal.getRightSetpoint());
    }

    Logger.recordOutput("Flywheels/Goal", goal);
    Logger.recordOutput("Flywheels/LeftSetpointRpm", goal.getLeftSetpoint());
    Logger.recordOutput("Flywheels/RightSetpointRpm", goal.getRightSetpoint());
    Logger.recordOutput("Flywheels/LeftRpm", inputs.leftVelocityRpm);
    Logger.recordOutput("Flywheels/RightRpm", inputs.rightVelocityRpm);
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
    return Math.abs(inputs.leftVelocityRpm - goal.getLeftSetpoint()) <= shooterTolerance.get()
        && Math.abs(inputs.rightVelocityRpm - goal.getRightSetpoint()) <= shooterTolerance.get();
  }

  public Command stop() {
    return runOnce(() -> setGoal(Goal.STOP)).withName("Flywheels Stop");
  }

  public Command idle() {
    return runOnce(() -> setGoal(Goal.IDLING)).withName("Flywheels Idle");
  }

  public Command shoot() {
    return startEnd(() -> setGoal(Goal.SHOOTING), () -> setGoal(Goal.IDLING))
        .withName("Flywheels Shooting");
  }

  public Command intake() {
    return startEnd(() -> setGoal(Goal.INTAKING), () -> setGoal(Goal.IDLING))
        .withName("Flywheels Intaking");
  }

  public Command eject() {
    return startEnd(() -> setGoal(Goal.EJECTING), () -> setGoal(Goal.IDLING))
        .withName("Flywheels Ejecting");
  }
}
