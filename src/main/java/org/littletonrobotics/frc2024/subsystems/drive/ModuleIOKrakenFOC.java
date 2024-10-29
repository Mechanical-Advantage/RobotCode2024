// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

public class ModuleIOKrakenFOC implements ModuleIO {
  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final AnalogInput turnAbsoluteEncoder;
  private final Rotation2d absoluteEncoderOffset;

  // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  private final StatusSignal<Double> turnPosition;
  private final Supplier<Rotation2d> turnAbsolutePosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnSupplyCurrent;
  private final StatusSignal<Double> turnTorqueCurrent;

  // Odometry Queues
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Controller Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);

  public ModuleIOKrakenFOC(ModuleConfig config) {
    // Init controllers and encoders from config constants
    driveTalon = new TalonFX(config.driveID(), "*");
    turnTalon = new TalonFX(config.turnID(), "*");
    turnAbsoluteEncoder = new AnalogInput(config.absoluteEncoderChannel());
    absoluteEncoderOffset = config.absoluteEncoderOffset();

    // Config Motors
    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turnTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    turnTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    turnTalonConfig.MotorOutput.Inverted =
        config.turnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Conversions affect getPosition()/setPosition() and getVelocity()
    driveTalonConfig.Feedback.SensorToMechanismRatio = moduleConstants.driveReduction();
    turnTalonConfig.Feedback.SensorToMechanismRatio = moduleConstants.turnReduction();
    turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Apply configs
    for (int i = 0; i < 4; i++) {
      boolean error = driveTalon.getConfigurator().apply(driveTalonConfig, 0.1) == StatusCode.OK;
      error = error && (turnTalon.getConfigurator().apply(turnTalonConfig, 0.1) == StatusCode.OK);
      if (!error) break;
    }

    // 250hz signals
    drivePosition = driveTalon.getPosition();
    turnPosition = turnTalon.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition, turnPosition);

    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, drivePosition);
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnPosition);

    // Get signals and set update rate
    // 100hz signals
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();
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
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrent,
        turnTorqueCurrent);

    // Reset turn position to absolute encoder position
    turnTalon.setPosition(turnAbsolutePosition.get().getRotations(), 1.0);

    // Optimize bus utilization
    driveTalon.optimizeBusUtilization(0, 1.0);
    turnTalon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.hasCurrentControl = true;
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
            .isOK();
    inputs.turnMotorConnected =
        BaseStatusSignal.refreshAll(
                turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent)
            .isOK();

    inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = turnAbsolutePosition.get();
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadsPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();

    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            .mapToDouble(
                signalValue -> Units.rotationsToRadians(signalValue) * driveConfig.wheelRadius())
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveVolts(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runTurnVolts(double volts) {
    turnTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCharacterization(double input) {
    driveTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    driveTalon.setControl(
        velocityTorqueCurrentFOC
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
            .withFeedForward(feedForward));
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    turnTalon.setControl(positionControl.withPosition(Units.radiansToRotations(angleRads)));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnTalonConfig.Slot0.kP = kP;
    turnTalonConfig.Slot0.kI = kI;
    turnTalonConfig.Slot0.kD = kD;
    turnTalon.getConfigurator().apply(turnTalonConfig, 0.01);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveTalonConfig) {
            driveTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            driveTalon.getConfigurator().apply(driveTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (turnTalonConfig) {
            turnTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            turnTalon.getConfigurator().apply(turnTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void stop() {
    driveTalon.setControl(neutralControl);
    turnTalon.setControl(neutralControl);
  }
}
