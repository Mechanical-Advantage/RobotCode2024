package org.littletonrobotics.frc2024.subsystems.superstructure.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public Feeder(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public void stop() {
        io.stop();
    }
}
