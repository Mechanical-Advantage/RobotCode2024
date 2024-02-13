package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import static org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", gains.ffkS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", gains.ffkV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", gains.ffkA());
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", gains.ffkG());
  private static final LoggedTunableNumber armVelocity =
      new LoggedTunableNumber("Arm/Velocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber armAcceleration =
      new LoggedTunableNumber("Arm/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber armToleranceDegreees =
      new LoggedTunableNumber("Arm/ToleranceDegrees", positionTolerance.getDegrees());
  private static final LoggedTunableNumber armLowerLimit =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", 15.0);
  private static final LoggedTunableNumber armUpperLimit =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", 90.0);
  private static LoggedTunableNumber armStowDegrees =
      new LoggedTunableNumber("Superstructure/ArmStowDegrees", 20.0);
  private static LoggedTunableNumber armStationIntakeDegrees =
      new LoggedTunableNumber("Superstructure/ArmStationIntakeDegrees", 45.0);
  private static LoggedTunableNumber armIntakeDegrees =
      new LoggedTunableNumber("Superstructure/ArmIntakeDegrees", 40.0);

  @RequiredArgsConstructor
  public enum Goal {
    STOW(() -> Units.degreesToRadians(armStowDegrees.get())),
    FLOOR_INTAKE(() -> Units.degreesToRadians(armIntakeDegrees.get())),
    STATION_INTAKE(() -> Units.degreesToRadians(armStowDegrees.get())),
    AIM(() -> RobotState.getInstance().getAimingParameters().armAngle().getRadians());

    private final DoubleSupplier armSetpointSupplier;

    private double getArmSetpointRads() {
      return armSetpointSupplier.getAsDouble();
    }
  }

  @Getter @Setter Goal goal;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private boolean homed = false;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
    io.setBrakeMode(true);

    setDefaultCommand(stowCommand());
  }

  @Override
  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setFF(kS.get(), kV.get(), kA.get(), kG.get()), kS, kV, kA, kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        constraints -> io.setProfileConstraints(constraints[0], constraints[1]),
        armVelocity,
        armAcceleration);

    io.runSetpoint(
        MathUtil.clamp(
            goal.getArmSetpointRads(),
            Units.degreesToRadians(armLowerLimit.get()),
            Units.degreesToRadians(armUpperLimit.get())));

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    Logger.recordOutput("Arm/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.armPositionRads);
  }

  @AutoLogOutput(key = "Arm/SetpointAngle")
  public Rotation2d getSetpoint() {
    return Rotation2d.fromRadians(goal.getArmSetpointRads());
  }

  @AutoLogOutput(key = "Arm/Homed")
  public boolean homed() {
    return homed;
  }

  @AutoLogOutput(key = "Arm/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.armPositionRads - goal.getArmSetpointRads())
        <= Units.degreesToRadians(armToleranceDegreees.get());
  }

  public Command stowCommand() {
    return runOnce(() -> setGoal(Goal.STOW)).andThen(Commands.idle()).withName("Arm Stow");
  }

  public Command intakeCommand() {
    return runOnce(() -> setGoal(Goal.FLOOR_INTAKE))
        .andThen(Commands.idle())
        .withName("Arm Intake");
  }

  public Command stationIntakeCommand() {
    return runOnce(() -> setGoal(Goal.STATION_INTAKE))
        .andThen(Commands.idle())
        .withName("Arm Station Intake");
  }

  public Command aimCommand() {
    return runOnce(() -> setGoal(Goal.AIM)).andThen(Commands.idle()).withName("Arm Aim");
  }

  public Command getStaticCurrent() {
    Timer timer = new Timer();
    return run(() -> io.runCurrent(0.5 * timer.get()))
        .beforeStarting(timer::restart)
        .until(() -> Math.abs(inputs.armVelocityRadsPerSec) >= Units.degreesToRadians(10))
        .andThen(() -> Logger.recordOutput("Arm/staticCurrent", inputs.armTorqueCurrentAmps[0]))
        .andThen(io::stop);
  }
}
