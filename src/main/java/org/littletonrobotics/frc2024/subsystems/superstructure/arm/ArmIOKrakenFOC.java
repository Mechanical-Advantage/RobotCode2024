// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import static org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.littletonrobotics.frc2024.util.Alert;

public class ArmIOKrakenFOC implements ArmIO {
  // Hardware
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final CANcoder absoluteEncoder;

  // Status Signals
  private final StatusSignal<Double> armInternalPositionRotations;
  private final StatusSignal<Double> armEncoderPositionRotations;
  private final StatusSignal<Double> armAbsolutePositionRotations;
  private final StatusSignal<Double> armVelocityRps;
  private final List<StatusSignal<Double>> armAppliedVoltage;
  private final List<StatusSignal<Double>> armOutputCurrent;
  private final List<StatusSignal<Double>> armTorqueCurrent;
  private final List<StatusSignal<Double>> armTempCelsius;

  // Control
  private final Slot0Configs controllerConfig;

  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  // Alerts
  private final Alert leaderMotorDisconnected =
      new Alert("Arm", "Leader Motor Disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Arm", "Follower Motor Disconnected!", Alert.AlertType.WARNING);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm", "Absolute Encoder Disconnected!", Alert.AlertType.WARNING);

  public ArmIOKrakenFOC() {
    leaderMotor = new TalonFX(leaderID, "canivore");
    followerMotor = new TalonFX(followerID, "canivore");
    followerMotor.setControl(new Follower(leaderID, true));
    absoluteEncoder = new CANcoder(armEncoderID, "canivore");

    // Arm Encoder Configs
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = armEncoderOffsetRotations;
    absoluteEncoder.getConfigurator().apply(armEncoderConfig, 1);

    // Leader motor configs
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.MotorOutput.Inverted =
        leaderInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderConfig.Feedback.FeedbackRemoteSensorID = armEncoderID;
    leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
    leaderConfig.Feedback.RotorToSensorRatio = reduction;

    // Set up controller
    controllerConfig = new Slot0Configs().withKP(gains.kP()).withKI(gains.kI()).withKD(gains.kD());
    leaderConfig.Slot0 = controllerConfig;

    // Follower configs
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Status signals
    armInternalPositionRotations = leaderMotor.getPosition();
    armEncoderPositionRotations = absoluteEncoder.getPosition();
    armAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
    armVelocityRps = leaderMotor.getVelocity();
    armAppliedVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());
    armOutputCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
    armTorqueCurrent = List.of(leaderMotor.getTorqueCurrent(), followerMotor.getTorqueCurrent());
    armTempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        armInternalPositionRotations,
        armEncoderPositionRotations,
        armAbsolutePositionRotations,
        armVelocityRps,
        armAppliedVoltage.get(0),
        armAppliedVoltage.get(1),
        armOutputCurrent.get(0),
        armOutputCurrent.get(1),
        armTorqueCurrent.get(0),
        armTorqueCurrent.get(1),
        armTempCelsius.get(0),
        armTempCelsius.get(1));
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.absoluteEncoderConnected = true;

    // Refresh signals & set alerts
    leaderMotorDisconnected.set(
        BaseStatusSignal.refreshAll(
                armInternalPositionRotations,
                armVelocityRps,
                armAppliedVoltage.get(0),
                armOutputCurrent.get(0),
                armTorqueCurrent.get(0),
                armTempCelsius.get(0))
            .isOK());
    followerMotorDisconnected.set(
        BaseStatusSignal.refreshAll(
                armAppliedVoltage.get(1),
                armOutputCurrent.get(1),
                armTorqueCurrent.get(1),
                armTempCelsius.get(1))
            .isOK());

    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(armEncoderPositionRotations, armAbsolutePositionRotations)
            .isOK();
    absoluteEncoderDisconnected.set(inputs.absoluteEncoderConnected);

    inputs.armPositionRads = Units.rotationsToRadians(armInternalPositionRotations.getValue());
    inputs.armEncoderPositionRads =
        Units.rotationsToRadians(armEncoderPositionRotations.getValue());
    inputs.armAbsoluteEncoderPositionRads =
        Units.rotationsToRadians(armAbsolutePositionRotations.getValue());
    inputs.armVelocityRadsPerSec = Units.rotationsToRadians(armVelocityRps.getValue());
    inputs.armAppliedVolts =
        armAppliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.armCurrentAmps =
        armOutputCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.armTorqueCurrentAmps =
        armTorqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.armTempCelcius =
        armTempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    leaderMotor.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(setpointRads))
            .withFeedForward(feedforward));
  }

  @Override
  public void runVolts(double volts) {
    leaderMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCurrent(double amps) {
    leaderMotor.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controllerConfig.kP = p;
    controllerConfig.kI = i;
    controllerConfig.kD = d;
    leaderMotor.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void setPosition(double positionRads) {
    leaderMotor.setPosition(Units.radiansToRotations(positionRads));
  }

  @Override
  public void stop() {
    leaderMotor.setControl(new NeutralOut());
  }
}
