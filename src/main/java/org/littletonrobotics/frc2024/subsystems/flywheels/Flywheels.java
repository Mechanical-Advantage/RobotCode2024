package org.littletonrobotics.frc2024.subsystems.flywheels;

import static org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.flywheels.FlywheelsIOInputsAutoLogged;
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

  @Getter @Setter private Goal goal = Goal.IDLE;
  private double characterizationVolts = 0.0;
  private boolean characterizing = false;

  public Flywheels(FlywheelsIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;

    setDefaultCommand(idleCommand());
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
    Logger.recordOutput("Flywheels/LeftSetpointRPM", goal.getLeftSetpoint());
    Logger.recordOutput("Flywheels/RightSetpointRPM", goal.getRightSetpoint());
    Logger.recordOutput("Flywheels/LeftRPM", inputs.leftVelocityRpm);
    Logger.recordOutput("Flywheels/RightRPM", inputs.rightVelocityRpm);
  }

  public void runLeftCharacterizationVolts(double volts) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationLeftVolts(volts);
  }

  public void runRightCharacterizationVolts(double volts) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationRightVolts(volts);
  }

  public double getLeftCharacterizationVelocity() {
    return inputs.leftVelocityRpm;
  }

  public double getRightCharacterizationVelocity() {
    return inputs.rightVelocityRpm;
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.leftVelocityRpm - goal.leftSetpoint.getAsDouble())
            <= shooterTolerance.get()
        && Math.abs(inputs.rightVelocityRpm - goal.rightSetpoint.getAsDouble())
            <= shooterTolerance.get();
  }

  public Command stopCommand() {
    return runOnce(() -> setGoal(Goal.STOP)).andThen(Commands.idle()).withName("Flywheels Stop");
  }

  public Command idleCommand() {
    return runOnce(() -> setGoal(Goal.IDLE)).andThen(Commands.idle()).withName("Flywheels Idle");
  }

  public Command shootingCommand() {
    return runOnce(() -> setGoal(Goal.SHOOTING))
        .andThen(Commands.idle())
        .withName("Flywheels Shooting");
  }

  public Command intakingCommand() {
    return runOnce(() -> setGoal(Goal.INTAKING))
        .andThen(Commands.idle())
        .withName("Flywheels Intaking");
  }
}
