package org.littletonrobotics.frc2024.subsystems.superstructure.feeder;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.FeederConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FeederIOKrakenFOC implements FeederIO {
    private final TalonFX motor;

    private StatusSignal<Double> position;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> appliedVoltage;
    private StatusSignal<Double> outputCurrent;

    public FeederIOKrakenFOC() {
        motor = new TalonFX(id, "canivore");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        motor.getConfigurator().apply(config);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        outputCurrent = motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, outputCurrent);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, outputCurrent);

        inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
        inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.outputCurrent = outputCurrent.getValueAsDouble();
    }

    @Override
    public void runVolts(double volts) {
        motor.setControl(new VoltageOut(volts).withEnableFOC(true));
    }

    @Override
    public void stop() {
        motor.setControl(new NeutralOut());
    }
}
