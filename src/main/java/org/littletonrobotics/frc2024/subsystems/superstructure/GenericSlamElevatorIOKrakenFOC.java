// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class GenericSlamElevatorIOKrakenFOC implements GenericSlamElevatorIO {
  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> velocityRps;
  private final StatusSignal<Double> appliedVoltage;
  private final StatusSignal<Double> supplyCurrent;
  private final StatusSignal<Double> torqueCurrent;
  private final StatusSignal<Double> tempCelsius;

  // Control
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  // Reduction to final sprocket
  private final double reduction;

  public GenericSlamElevatorIOKrakenFOC(
      int id, String bus, int currentLimitAmps, boolean invert, double reduction) {
    talon = new TalonFX(id, bus);
    this.reduction = reduction;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    positionRotations = talon.getPosition();
    velocityRps = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    talon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(GenericSlamElevatorIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();
    inputs.positionRads =
        Units.rotationsToRadians(positionRotations.getValueAsDouble()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocityRps.getValueAsDouble()) / reduction;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runCurrent(double amps) {
    talon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
