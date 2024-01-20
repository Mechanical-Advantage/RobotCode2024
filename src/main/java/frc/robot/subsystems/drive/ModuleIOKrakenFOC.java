// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;
import java.util.function.Supplier;

public class ModuleIOKrakenFOC implements ModuleIO {
  // Reseed relative encoder to absolute encoder value
  private static final int reseedRelativeEncoderCounts = 100;
  private int currentReseedCount = reseedRelativeEncoderCounts;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnPosition;
  private final Supplier<Rotation2d> turnAbsolutePosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final boolean isTurnMotorInverted = true;

  // queues
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final AnalogInput turnAbsoluteEncoder;
  private final Rotation2d absoluteEncoderOffset;

  // Setpoint control
  private final VelocityTorqueCurrentFOC driveVelocityControl =
      new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
  private final PositionTorqueCurrentFOC turnPositionControl =
      new PositionTorqueCurrentFOC(0.0, 0.0, 0.0, 1, false, false, false);
  private final VoltageOut driveVoltageControl = new VoltageOut(0.0).withEnableFOC(true);
  private final NeutralOut driveBrake = new NeutralOut();
  private final NeutralOut turnBrake = new NeutralOut();
  private Mode currentMode = Mode.SETPOINT;
  private double driveSpeedSetpoint = 0.0;
  private Rotation2d turnAngleSetpoint = new Rotation2d();
  private double driveVoltage = 0.0;

  public ModuleIOKrakenFOC(ModuleConfig config) {
    // init controllers and encoders from config constants
    driveTalon = new TalonFX(config.driveID());
    turnTalon = new TalonFX(config.turnID());
    turnAbsoluteEncoder = new AnalogInput(config.absoluteEncoderChannel());
    absoluteEncoderOffset = config.absoluteEncoderOffset();

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.Voltage.PeakForwardVoltage = 12.0;
    driveConfig.Voltage.PeakReverseVoltage = -12.0;
    // TUNE PID CONSTANTS
    driveConfig.Slot0.kP = moduleConstants.driveKp();
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = moduleConstants.drivekD();
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.Voltage.PeakForwardVoltage = 12.0;
    turnConfig.Voltage.PeakReverseVoltage = -12.0;
    // TUNE PID CONSTANTS
    turnConfig.Slot0.kP = moduleConstants.turnKp();
    turnConfig.Slot0.kI = 0.0;
    turnConfig.Slot0.kD = moduleConstants.turnkD();
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 30;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -30;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // Map rotation of motor shaft to 1 rotation of turn shaft
    turnConfig.Feedback.SensorToMechanismRatio = moduleConstants.turnReduction();

    // Apply configs
    for (int i = 0; i < 4; i++) {
      boolean error = driveTalon.getConfigurator().apply(driveConfig, 0.1) == StatusCode.OK;
      setDriveBrakeMode(true);
      error = error && (turnTalon.getConfigurator().apply(turnConfig, 0.1) == StatusCode.OK);
      setTurnBrakeMode(true);
      if (!error) break;
    }

    // Get signals and set update rate
    // 50hz signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnPosition = turnTalon.getPosition();
    turnAbsolutePosition =
        () ->
            Rotation2d.fromRadians(
                    turnAbsoluteEncoder.getVoltage()
                        / RobotController.getVoltage5V()
                        * 2.0
                        * Math.PI)
                .minus(absoluteEncoderOffset);
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // 250hz signals
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition, turnPosition);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble())
            / moduleConstants.driveReduction();
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble())
            / moduleConstants.driveReduction();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition = turnAbsolutePosition.get();
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / moduleConstants.turnReduction());
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / moduleConstants.turnReduction();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            .mapToDouble(
                signal ->
                    Units.rotationsToRadians(signal)
                        * wheelRadius
                        / moduleConstants.driveReduction())
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((signal) -> Rotation2d.fromRotations(signal / moduleConstants.turnReduction()))
            .toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void periodic() {
    // Only reseed if turn abs encoder is online
    if (++currentReseedCount >= reseedRelativeEncoderCounts
        && turnAbsoluteEncoder.getVoltage() != 0.0) {
      // heading of wheel --> turn motor shaft rotations
      // timeout of 20 ms
      boolean passed =
          turnTalon.setPosition(
                  Units.radiansToRotations(turnAbsolutePosition.get().getRadians())
                      * moduleConstants.turnReduction(),
                  0.02)
              == StatusCode.OK;
      // redo next cycle if did not pass
      currentReseedCount = passed ? 0 : currentReseedCount;
    }

    // Control motors
    if (currentMode == Mode.NEUTRAL) {
      driveTalon.setControl(driveBrake);
      turnTalon.setControl(turnBrake);
    } else {
      if (currentMode == Mode.SETPOINT) {
        driveTalon.setControl(driveVelocityControl.withVelocity(driveSpeedSetpoint));
      } else if (currentMode == Mode.CHARACTERIZATION) {
        driveTalon.setControl(driveVoltageControl.withOutput(driveVoltage));
      }
      turnTalon.setControl(turnPositionControl.withPosition(turnAngleSetpoint.getRotations()));
    }
  }

  @Override
  public void runSetpoint(SwerveModuleState state) {
    currentMode = Mode.SETPOINT;
    driveSpeedSetpoint = state.speedMetersPerSecond / wheelRadius;
    turnAngleSetpoint = state.angle;
  }

  @Override
  public void runCharacterization(double volts) {
    currentMode = Mode.CHARACTERIZATION;
    driveVoltage = volts;
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    currentMode = Mode.NEUTRAL;
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(
        Units.rotationsToRadians(turnPosition.getValueAsDouble())
            / moduleConstants.turnReduction());
  }

  @Override
  public double getPositionMeters() {
    return Units.rotationsToRadians(drivePosition.getValueAsDouble())
        / moduleConstants.driveReduction()
        * wheelRadius;
  }

  @Override
  public double getVelocityMetersPerSec() {
    return Units.rotationsToRadians(driveVelocity.getValueAsDouble())
        / moduleConstants.driveReduction()
        * wheelRadius;
  }

  @Override
  public double getCharacterizationVelocity() {
    return Units.rotationsToRadians(driveVelocity.getValueAsDouble())
        / moduleConstants.driveReduction();
  }

  private enum Mode {
    SETPOINT,
    CHARACTERIZATION,
    NEUTRAL;
  }
}
