// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import static org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.frc2024.util.EqualsUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.frc2024.util.NoteVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", gains.ffkS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", gains.ffkV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", gains.ffkA());
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Arm/Velocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Arm/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber slowVelocity =
      new LoggedTunableNumber("Arm/SlowVelocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber slowAcceleration =
      new LoggedTunableNumber("Arm/SlowAcceleration", 2.0);
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", maxAngle.getDegrees());

  @RequiredArgsConstructor
  public enum Goal {
    STOP(() -> 0),
    FLOOR_INTAKE(new LoggedTunableNumber("Arm/IntakeDegrees", 18.0)),
    STATION_INTAKE(new LoggedTunableNumber("Arm/StationIntakeDegrees", 45.0)),
    AIM(() -> RobotState.getInstance().getAimingParameters().armAngle().getDegrees()),
    STOW(new LoggedTunableNumber("Arm/StowDegrees", 0.0)),
    AMP(new LoggedTunableNumber("Arm/AmpDegrees", 110.0)),
    SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees", 55.0)),
    PODIUM(new LoggedTunableNumber("Arm/PodiumDegrees", 30.0)),
    PREPARE_CLIMB(new LoggedTunableNumber("Arm/PrepareClimbDegrees", 105.0)),
    CLIMB(new LoggedTunableNumber("Arm/ClimbDegrees", 90.0)),
    CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));

    private final DoubleSupplier armSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  @Getter @Setter private Goal goal = Goal.STOW;
  private Goal lastGoal = goal;
  private boolean characterizing = false;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private TrapezoidProfile motionProfile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private ArmFeedforward ff;

  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final ArmVisualizer goalVisualizer;

  private final Alert leaderMotorDisconnected =
      new Alert("Arm leader motor disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Arm follower motor disconnected!", Alert.AlertType.WARNING);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm absolute encoder disconnected!", Alert.AlertType.WARNING);

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private boolean brakeModeEnabled = true;

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

    motionProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Set up visualizers
    NoteVisualizer.setArmAngleSupplier(() -> Rotation2d.fromRadians(inputs.armPositionRads));
    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);
  }

  public void setOverrides(BooleanSupplier disableOverride, BooleanSupplier coastOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    coastSupplier = coastOverride;
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Set alerts
    leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
    followerMotorDisconnected.set(!inputs.followerMotorConnected);
    absoluteEncoderDisconnected.set(!inputs.absoluteEncoderConnected);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        constraints ->
            motionProfile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(constraints[0], constraints[1])),
        maxVelocity,
        maxAcceleration);

    // Check if disabled
    if (disableSupplier.getAsBoolean() || goal == Goal.STOP) {
      io.stop();
      // Reset profile when disabled
      setpointState = new TrapezoidProfile.State(inputs.armPositionRads, 0);
    }

    // Set coast mode with override
    setBrakeMode(!coastSupplier.getAsBoolean() || DriverStation.isEnabled());

    if (lastGoal != goal && goal != Goal.STOP) {
      // Update profile constraints when prepare climb
      if (goal == Goal.PREPARE_CLIMB) {
        motionProfile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(slowVelocity.get(), slowAcceleration.get()));
      } else {
        motionProfile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
      }
      lastGoal = goal;
    }

    // Don't run profile when characterizing, coast mode, or disabled
    if (!characterizing
        && brakeModeEnabled
        && !disableSupplier.getAsBoolean()
        && goal != Goal.STOP) {
      // Run closed loop
      setpointState =
          motionProfile.calculate(
              Constants.loopPeriodSecs,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      goal.getRads(),
                      Units.degreesToRadians(lowerLimitDegrees.get()),
                      Units.degreesToRadians(upperLimitDegrees.get())),
                  0.0));

      io.runSetpoint(
          setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
    }

    // Logs
    measuredVisualizer.update(inputs.armPositionRads);
    setpointVisualizer.update(setpointState.position);
    goalVisualizer.update(goal.getRads());
    Logger.recordOutput("Arm/GoalAngle", goal.getRads());
    Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
    Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Superstructure/Arm/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goal.getRads(), 1e-3);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void runCharacterization(double amps) {
    characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.armVelocityRadsPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }
}
