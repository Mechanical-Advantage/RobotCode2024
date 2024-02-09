package org.littletonrobotics.frc2024.subsystems.superstructure.feeder;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.FeederConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim sim = new FlywheelSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            0.01);

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        FeederIO.super.updateInputs(inputs);
    }

    @Override
    public void runVolts(double volts) {
        FeederIO.super.runVolts(volts);
    }

    @Override
    public void stop() {
        FeederIO.super.stop();
    }
}
