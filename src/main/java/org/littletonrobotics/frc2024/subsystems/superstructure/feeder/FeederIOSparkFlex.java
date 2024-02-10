package org.littletonrobotics.frc2024.subsystems.superstructure.feeder;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.FeederConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class FeederIOSparkFlex implements FeederIO {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;

    public FeederIOSparkFlex() {
        motor = new CANSparkFlex(id, CANSparkBase.MotorType.kBrushless);

        motor.setInverted(inverted);
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
        inputs.velocityRadsPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.outputCurrent = motor.getOutputCurrent();
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
