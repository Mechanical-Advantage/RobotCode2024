package org.littletonrobotics.frc2024.subsystems.superstructure.feeder;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.FeederConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim sim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), reduction, 0.01);

    private double appliedVoltage = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        sim.update(0.02);
        inputs.positionRads += sim.getAngularVelocityRadPerSec() * 0.02;
        inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.outputCurrent = sim.getCurrentDrawAmps();
    }

    @Override
    public void runVolts(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }
}
