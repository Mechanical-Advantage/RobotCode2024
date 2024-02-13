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
  private static LoggedTunableNumber shootingLeftRPM =
      new LoggedTunableNumber("Superstructure/ShootingLeftRPM", 6000.0);
  private static LoggedTunableNumber shootingRightRPM =
      new LoggedTunableNumber("Superstructure/ShootingRightRPM", 4000.0);
  private static LoggedTunableNumber idleLeftRPM =
      new LoggedTunableNumber("Superstructure/IdleLeftRPM", 200.0);
  private static LoggedTunableNumber idleRightRPM =
      new LoggedTunableNumber("Superstructure/IdleRightRPM", 200.0);

  private static LoggedTunableNumber intakingLeftRPM =
      new LoggedTunableNumber("Superstructure/IntakingLeftRPM", -2000.0);
  private static LoggedTunableNumber intakingRightRPM =
      new LoggedTunableNumber("Superstructure/IntakingRightRPM", -2000.0);
  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Flywheels/ToleranceRPM", config.toleranceRPM());

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  @RequiredArgsConstructor
  public enum Goal {
    STOP(() -> 0.0, () -> 0.0),
    IDLE(idleLeftRPM, idleRightRPM),
    SHOOTING(shootingLeftRPM, shootingRightRPM),
    INTAKING(intakingLeftRPM, intakingRightRPM),
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

  @Getter private Goal goal = Goal.IDLE;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
    setDefaultCommand(runOnce(() -> goal = Goal.IDLE).withName("FlywheelsIdle"));
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
      goal = Goal.STOP;
    }

    switch (goal) {
      case STOP -> io.stop();
      case CHARACTERIZING -> {} // Handled by runCharacterizationVolts
      default -> io.runVelocity(goal.getLeftSetpoint(), goal.getRightSetpoint());
    }

    Logger.recordOutput("Flywheels/Goal", goal);
    Logger.recordOutput("Flywheels/LeftSetpointRPM", goal.getLeftSetpoint());
    Logger.recordOutput("Flywheels/RightSetpointRPM", goal.getRightSetpoint());
    Logger.recordOutput("Flywheels/LeftRPM", inputs.leftVelocityRpm);
    Logger.recordOutput("Flywheels/RightRPM", inputs.rightVelocityRpm);
  }

  public void runCharacterizationVolts(double volts) {
    goal = Goal.CHARACTERIZING;
    io.runCharacterizationLeftVolts(volts);
    io.runCharacterizationRightVolts(volts);
  }

  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.leftVelocityRpm - goal.leftSetpoint.getAsDouble())
            <= shooterTolerance.get()
        && Math.abs(inputs.rightVelocityRpm - goal.rightSetpoint.getAsDouble())
            <= shooterTolerance.get();
  }

  public Command shootCommand() {
    return startEnd(() -> goal = Goal.SHOOTING, () -> goal = Goal.IDLE).withName("FlywheelsShoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> goal = Goal.INTAKING, () -> goal = Goal.IDLE).withName("FlywheelsIntake");
  }
}
