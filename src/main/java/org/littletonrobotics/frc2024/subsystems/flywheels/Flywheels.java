// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.flywheels;

import static org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.Robot;
import org.littletonrobotics.frc2024.RobotState;
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
      new LoggedTunableNumber("Flywheels/ShootingLeftRpm", 7000.0);
  private static final LoggedTunableNumber shootingRightRpm =
      new LoggedTunableNumber("Flywheels/ShootingRightRpm", 4500.0);
  private static final LoggedTunableNumber prepareShootMultiplier =
      new LoggedTunableNumber("Flywheels/PrepareShootMultiplier", 0.75);
  private static final LoggedTunableNumber intakingRpm =
      new LoggedTunableNumber("Flywheels/IntakingRpm", -3000.0);
  private static final LoggedTunableNumber ejectingRpm =
      new LoggedTunableNumber("Flywheels/EjectingRpm", 1000.0);
  private static final LoggedTunableNumber poopingRpm =
      new LoggedTunableNumber("Flywheels/PoopingRpm", 3000.0);
  private static final LoggedTunableNumber superPoopRpm =
      new LoggedTunableNumber("Flywheels/SuperPoopingRpm", 3000.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;
  @Setter private BooleanSupplier prepareShootSupplier = () -> false;

  // Disconnected alerts
  private final Alert leftDisconnected =
      new Alert("Left flywheel disconnected!", Alert.AlertType.WARNING);
  private final Alert rightDisconnected =
      new Alert("Right flywheel disconnected!", Alert.AlertType.WARNING);

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    SHOOT(shootingLeftRpm, shootingRightRpm),
    INTAKE(intakingRpm, intakingRpm),
    EJECT(ejectingRpm, ejectingRpm),
    POOP(poopingRpm, poopingRpm),
    SUPER_POOP(superPoopRpm, superPoopRpm),
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

  public enum IdleMode {
    TELEOP,
    AUTO
  }

  @Getter
  @AutoLogOutput(key = "Flywheels/Goal")
  private Goal goal = Goal.IDLE;

  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.leftSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.rightSupplyCurrentAmps) > 50.0;
  }

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
    Robot.totalCurrent += inputs.leftSupplyCurrentAmps + inputs.rightSupplyCurrentAmps;

    // Set alerts
    leftDisconnected.set(!inputs.leftMotorConnected);
    rightDisconnected.set(!inputs.rightMotorConnected);

    // Check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> ff = new SimpleMotorFeedforward(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          leftProfile.setMaxAcceleration(maxAcceleration.get());
          rightProfile.setMaxAcceleration(maxAcceleration.get());
        },
        maxAcceleration);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Check if profile needs to be reset
    if (!closedLoop && wasClosedLoop) {
      leftProfile.reset();
      rightProfile.reset();
      wasClosedLoop = false;
    }

    // Get goal
    double leftGoal = goal.getLeftGoal();
    double rightGoal = goal.getRightGoal();
    boolean idlePrepareShoot = goal == Goal.IDLE && prepareShootSupplier.getAsBoolean();
    if (idlePrepareShoot) {
      leftGoal = Goal.SHOOT.getLeftGoal() * prepareShootMultiplier.get();
      rightGoal = Goal.SHOOT.getRightGoal() * prepareShootMultiplier.get();
    }

    // Run to setpoint
    if (closedLoop || idlePrepareShoot) {
      // Update goals
      leftProfile.setGoal(leftGoal);
      rightProfile.setGoal(rightGoal);
      double leftSetpoint = leftProfile.calculateSetpoint();
      double rightSetpoint = rightProfile.calculateSetpoint();
      io.runVelocity(
          leftSetpoint, rightSetpoint, ff.calculate(leftSetpoint), ff.calculate(rightSetpoint));
      RobotState.getInstance().setFlywheelAccelerating(!atGoal() || isDrawingHighCurrent());
    } else if (goal == Goal.IDLE) {
      RobotState.getInstance().setFlywheelAccelerating(false);
      io.stop();
    }

    Logger.recordOutput("Flywheels/SetpointLeftRpm", leftProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/SetpointRightRpm", rightProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/GoalLeftRpm", leftGoal);
    Logger.recordOutput("Flywheels/GoalRightRpm", rightGoal);
  }

  /** Set the current goal of the flywheel */
  private void setGoal(Goal goal) {
    if (goal == Goal.CHARACTERIZING || goal == Goal.IDLE) {
      wasClosedLoop = closedLoop;
      closedLoop = false;
      this.goal = goal;
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

  /** Runs flywheels at the commanded voltage or amps. */
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
    return goal == Goal.IDLE
        || (leftProfile.getCurrentSetpoint() == goal.getLeftGoal()
            && rightProfile.getCurrentSetpoint() == goal.getRightGoal());
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Intake");
  }

  public Command poopCommand() {
    return startEnd(() -> setGoal(Goal.POOP), () -> setGoal(Goal.IDLE)).withName("Flywheels Poop");
  }

  public Command superPoopCommand() {
    return startEnd(() -> setGoal(Goal.SUPER_POOP), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Super Poop");
  }
}
