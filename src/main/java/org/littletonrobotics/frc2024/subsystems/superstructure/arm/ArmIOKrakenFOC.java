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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class ArmIOKrakenFOC implements ArmIO {
  // Hardware
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;
  private final CANcoder absoluteEncoder;

  // Status Signals
  private final StatusSignal<Double> internalPositionRotations;
  private final StatusSignal<Double> absolutePositionRotations;
  private final StatusSignal<Double> velocityRps;
  private final List<StatusSignal<Double>> appliedVoltage;
  private final List<StatusSignal<Double>> supplyCurrent;
  private final List<StatusSignal<Double>> torqueCurrent;
  private final List<StatusSignal<Double>> tempCelsius;

  // Control
  private final Slot0Configs controllerConfig;
  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  public ArmIOKrakenFOC() {
    leaderTalon = new TalonFX(leaderID, "*");
    followerTalon = new TalonFX(followerID, "*");
    followerTalon.setControl(new Follower(leaderID, true));
    absoluteEncoder = new CANcoder(armEncoderID, "*");

    // Arm Encoder Configs
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoder.getConfigurator().apply(armEncoderConfig, 1);

    // Leader motor configs
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.MotorOutput.Inverted =
        leaderInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set up controller
    controllerConfig = new Slot0Configs().withKP(gains.kP()).withKI(gains.kI()).withKD(gains.kD());
    leaderConfig.Slot0 = controllerConfig;

    // Follower configs
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Status signals
    internalPositionRotations = leaderTalon.getPosition();
    absolutePositionRotations = absoluteEncoder.getAbsolutePosition();
    velocityRps = leaderTalon.getVelocity();
    appliedVoltage = List.of(leaderTalon.getMotorVoltage(), followerTalon.getMotorVoltage());
    supplyCurrent = List.of(leaderTalon.getSupplyCurrent(), followerTalon.getSupplyCurrent());
    torqueCurrent = List.of(leaderTalon.getTorqueCurrent(), followerTalon.getTorqueCurrent());
    tempCelsius = List.of(leaderTalon.getDeviceTemp(), followerTalon.getDeviceTemp());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
            internalPositionRotations,
            absolutePositionRotations,
            velocityRps,
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));

    // Optimize bus utilization
    leaderTalon.optimizeBusUtilization(1.0);
    followerTalon.optimizeBusUtilization(1.0);

    // Set position of talon
    absolutePositionRotations.waitForUpdate(1.0);
    leaderTalon.setPosition(
        Units.radiansToRotations(
                MathUtil.angleModulus(
                    Units.rotationsToRadians(absolutePositionRotations.getValueAsDouble())
                        - armEncoderOffsetRads))
            * reduction,
        1.0);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                        internalPositionRotations,
                        velocityRps,
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
            .isOK();
    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1))
            .isOK();
    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(absolutePositionRotations).isOK();

    inputs.positionRads =
        Units.rotationsToRadians(internalPositionRotations.getValueAsDouble() / reduction);
    inputs.absoluteEncoderPositionRads =
        Units.rotationsToRadians(absolutePositionRotations.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocityRps.getValue() / reduction);
    inputs.appliedVolts =
        appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.torqueCurrentAmps =
        torqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.tempCelcius =
        tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    leaderTalon.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(setpointRads) * reduction)
            .withFeedForward(feedforward));
  }

  @Override
  public void runVolts(double volts) {
    leaderTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCurrent(double amps) {
    leaderTalon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controllerConfig.kP = p;
    controllerConfig.kI = i;
    controllerConfig.kD = d;
    leaderTalon.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void setPosition(double positionRads) {
    //    leaderTalon.setPosition(Units.radiansToRotations(positionRads * reduction), 0.2); TODO:
    // Figure this out.
  }

  @Override
  public void stop() {
    leaderTalon.setControl(new NeutralOut());
  }
}
