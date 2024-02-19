// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.flywheels;

import static org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2024.util.Alert;

public class FlywheelsIOKrakenFOC implements FlywheelsIO {
  // Hardware
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  // Status Signals
  private final StatusSignal<Double> leftPosition;
  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> leftAppliedVolts;
  private final StatusSignal<Double> leftTorqueCurrent;
  private final StatusSignal<Double> leftTempCelsius;
  private final StatusSignal<Double> rightPosition;
  private final StatusSignal<Double> rightVelocity;
  private final StatusSignal<Double> rightAppliedVolts;
  private final StatusSignal<Double> rightTorqueCurrent;
  private final StatusSignal<Double> rightTempCelsius;

  // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  // Connected Alerts
  private final Alert leftDisconnectedAlert =
      new Alert("Flywheels", "Left disconnected!", Alert.AlertType.WARNING);
  private final Alert rightDisconnectedAlert =
      new Alert("Flywheels", "Right disconnected!", Alert.AlertType.WARNING);

  public FlywheelsIOKrakenFOC() {
    leftTalon = new TalonFX(flywheelConfig.leftID(), "rio");
    rightTalon = new TalonFX(flywheelConfig.rightID(), "rio");

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = flywheelConfig.reduction();

    // Controller config;
    controllerConfig.kP = gains.kP();
    controllerConfig.kI = gains.kI();
    controllerConfig.kD = gains.kD();
    controllerConfig.kS = gains.kS();
    controllerConfig.kV = gains.kV();
    controllerConfig.kA = gains.kA();

    // Apply configs
    leftTalon.getConfigurator().apply(config, 1.0);
    rightTalon.getConfigurator().apply(config, 1.0);
    leftTalon.getConfigurator().apply(controllerConfig, 1.0);
    rightTalon.getConfigurator().apply(controllerConfig, 1.0);

    // Set signals
    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftTorqueCurrent = leftTalon.getTorqueCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();
    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightTorqueCurrent = rightTalon.getTorqueCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftTorqueCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightTorqueCurrent,
        rightTempCelsius);
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    leftDisconnectedAlert.set(
        !BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftAppliedVolts, leftTorqueCurrent, leftTempCelsius)
            .isOK());
    rightDisconnectedAlert.set(
        !BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightTorqueCurrent,
                rightTempCelsius)
            .isOK());

    inputs.leftPositionRads = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftOutputCurrent = leftTorqueCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();

    inputs.rightPositionRads = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightOutputCurrent = rightTorqueCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftTalon.setControl(voltageControl.withOutput(leftVolts));
    rightTalon.setControl(voltageControl.withOutput(rightVolts));
  }

  @Override
  public void stop() {
    leftTalon.setControl(neutralControl);
    rightTalon.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double leftRpm, double rightRpm) {
    leftTalon.setControl(velocityControl.withVelocity(leftRpm / 60.0));
    rightTalon.setControl(velocityControl.withVelocity(rightRpm / 60.0));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    leftTalon.getConfigurator().apply(controllerConfig);
    rightTalon.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    controllerConfig.kS = kS;
    controllerConfig.kV = kV;
    controllerConfig.kA = kA;
    leftTalon.getConfigurator().apply(controllerConfig);
    rightTalon.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runCharacterizationRight(double input) {
    rightTalon.setControl(currentControl.withOutput(input));
  }
}
