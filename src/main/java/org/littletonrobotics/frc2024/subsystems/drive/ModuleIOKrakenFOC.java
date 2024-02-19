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
import com.ctre.phoenix6.configs.Slot0Configs;
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
  private final Slot0Configs driveFeedbackConfig = new Slot0Configs();
  private final Slot0Configs turnFeedbackConfig = new Slot0Configs();

  // Control
  private final VoltageOut driveVoltage = new VoltageOut(0).withUpdateFreqHz(0);
  private final VoltageOut turnVoltage = new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC driveCurrent = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC turnCurrent = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC driveVelocityControl =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC turnPositionControl =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final NeutralOut driveNeutral = new NeutralOut().withUpdateFreqHz(0);
  private final NeutralOut turnNeutral = new NeutralOut().withUpdateFreqHz(0);

  public ModuleIOKrakenFOC(ModuleConfig config) {
    // Init controllers and encoders from config constants
    driveTalon = new TalonFX(config.driveID(), "canivore");
    turnTalon = new TalonFX(config.turnID(), "canivore");
    turnAbsoluteEncoder = new AnalogInput(config.absoluteEncoderChannel());
    absoluteEncoderOffset = config.absoluteEncoderOffset();

    // Config Motors
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.Voltage.PeakForwardVoltage = 12.0;
    driveConfig.Voltage.PeakReverseVoltage = -12.0;

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfig.Voltage.PeakForwardVoltage = 12.0;
    turnConfig.Voltage.PeakReverseVoltage = -12.0;
    turnConfig.MotorOutput.Inverted =
        config.turnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Conversions affect getPosition()/setPosition() and getVelocity()
    driveConfig.Feedback.SensorToMechanismRatio = moduleConstants.driveReduction();
    turnConfig.Feedback.SensorToMechanismRatio = moduleConstants.turnReduction();
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Apply configs
    for (int i = 0; i < 4; i++) {
      boolean error = driveTalon.getConfigurator().apply(driveConfig, 0.1) == StatusCode.OK;
      setDriveBrakeMode(true);
      error = error && (turnTalon.getConfigurator().apply(turnConfig, 0.1) == StatusCode.OK);
      setTurnBrakeMode(true);
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
    driveTalon.optimizeBusUtilization(1.0);
    turnTalon.optimizeBusUtilization(1.0);
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

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = turnAbsolutePosition.get();
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
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
    driveTalon.setControl(driveVoltage.withOutput(volts));
  }

  @Override
  public void runTurnVolts(double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void runDriveCurrent(double current) {
    driveTalon.setControl(driveCurrent.withOutput(current));
  }

  @Override
  public void runTurnCurrent(double current) {
    turnTalon.setControl(turnCurrent.withOutput(current));
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    driveTalon.setControl(
        driveVelocityControl
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
            .withFeedForward(feedForward));
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    turnTalon.setControl(turnPositionControl.withPosition(Units.radiansToRotations(angleRads)));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedbackConfig.kP = kP;
    driveFeedbackConfig.kI = kI;
    driveFeedbackConfig.kD = kD;
    driveTalon.getConfigurator().apply(driveFeedbackConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedbackConfig.kP = kP;
    turnFeedbackConfig.kI = kI;
    turnFeedbackConfig.kD = kD;
    turnTalon.getConfigurator().apply(turnFeedbackConfig, 0.01);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void stop() {
    driveTalon.setControl(driveNeutral);
    turnTalon.setControl(turnNeutral);
  }
}
