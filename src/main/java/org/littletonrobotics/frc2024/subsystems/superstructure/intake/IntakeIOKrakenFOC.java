package org.littletonrobotics.frc2024.subsystems.superstructure.intake;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeIOKrakenFOC implements IntakeIO {
    private final TalonFX motor;

    private final TalonFX otherMotor;
    private StatusSignal<Double> velocityRadsPerSec;
    private StatusSignal<Double> positionRads;
    private StatusSignal<Double> appliedVoltage;
    private StatusSignal<Double> currentAmps;

    public IntakeIOKrakenFOC() {
        motor = new TalonFX(2, "canivore");
        otherMotor = new TalonFX(3);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);

        velocityRadsPerSec = motor.getVelocity();
        positionRads = motor.getPosition();
        appliedVoltage = motor.getMotorVoltage();
        currentAmps = motor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, velocityRadsPerSec, positionRads, appliedVoltage, currentAmps);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocityRadsPerSec, positionRads, appliedVoltage, currentAmps);

        inputs.velocityRadsPerSec =
                Units.rotationsToRadians(velocityRadsPerSec.getValueAsDouble()) / reduction;
        inputs.positionRads = Units.rotationsToRadians(positionRads.getValueAsDouble()) / reduction;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(new VoltageOut(volts).withEnableFOC(true));
        otherMotor.setControl(new VoltageOut(volts).withEnableFOC(true));
    }

    @Override
    public void stop() {
        motor.setControl(new NeutralOut());
        otherMotor.setControl(new NeutralOut());
    }
}
