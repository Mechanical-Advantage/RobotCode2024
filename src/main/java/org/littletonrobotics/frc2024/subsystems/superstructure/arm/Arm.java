// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import static org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
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
      new LoggedTunableNumber("Arm/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber armUpperLimit =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", maxAngle.getDegrees());
  private static final LoggedTunableNumber armStowDegrees =
      new LoggedTunableNumber("Superstructure/ArmStowDegrees", 20.0);
  private static final LoggedTunableNumber armStationIntakeDegrees =
      new LoggedTunableNumber("Superstructure/ArmStationIntakeDegrees", 45.0);
  private static final LoggedTunableNumber armIntakeDegrees =
      new LoggedTunableNumber("Superstructure/ArmIntakeDegrees", 40.0);

  @RequiredArgsConstructor
  public enum Goal {
    FLOOR_INTAKE(() -> Units.degreesToRadians(armIntakeDegrees.get())),
    STATION_INTAKE(() -> Units.degreesToRadians(armStationIntakeDegrees.get())),
    AIM(() -> RobotState.getInstance().getAimingParameters().armAngle().getRadians()),
    STOW(() -> Units.degreesToRadians(armStowDegrees.get()));

    private final DoubleSupplier armSetpointSupplier;

    private double getRads() {
      return armSetpointSupplier.getAsDouble();
    }
  }

  @Getter @Setter private Goal goal = Goal.STOW;
  private boolean characterizing = false;
  private boolean homed = false;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private TrapezoidProfile motionProfile;
  private ArmFeedforward ff;

  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final ArmVisualizer goalVisualizer;

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

    motionProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(armVelocity.get(), armAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    measuredVisualizer = new ArmVisualizer("measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("setpoint", Color.kGreen);
    goalVisualizer = new ArmVisualizer("goal", Color.kBlue);
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kV.get(), kA.get(), kG.get()),
        kS,
        kV,
        kA,
        kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        constraints ->
            motionProfile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(constraints[0], constraints[1])),
        armVelocity,
        armAcceleration);

    if (!characterizing) {
      // Run closed loop
      var output =
          motionProfile.calculate(
              0.02,
              new TrapezoidProfile.State(inputs.armPositionRads, inputs.armVelocityRadsPerSec),
              new TrapezoidProfile.State(goal.getRads(), 0.0));

      io.runSetpoint(output.position, ff.calculate(output.position, output.velocity));

      // Logs
      Logger.recordOutput("Arm/SetpointAngle", output.position);
      setpointVisualizer.update(Rotation2d.fromRadians(output.position));
      goalVisualizer.update(Rotation2d.fromRadians(goal.getRads()));
    }

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    measuredVisualizer.update(Rotation2d.fromRadians(inputs.armPositionRads));
    Logger.recordOutput("Arm/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Arm/GoalAngle")
  public Rotation2d getSetpoint() {
    return Rotation2d.fromRadians(goal.getRads());
  }

  @AutoLogOutput(key = "Arm/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.armPositionRads - goal.getRads())
        <= Units.degreesToRadians(armToleranceDegreees.get());
  }

  // public Command getStaticCurrent() {
  //   Timer timer = new Timer();
  //   return run(() -> io.runCurrent(0.5 * timer.get()))
  //       .beforeStarting(timer::restart)
  //       .until(() -> Math.abs(inputs.armVelocityRadsPerSec) >= Units.degreesToRadians(10))
  //       .andThen(() -> Logger.recordOutput("Arm/staticCurrent", inputs.armTorqueCurrentAmps[0]))
  //       .andThen(io::stop);
  // }
}
