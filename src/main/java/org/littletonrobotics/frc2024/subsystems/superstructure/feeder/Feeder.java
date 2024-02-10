package org.littletonrobotics.frc2024.subsystems.superstructure.feeder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private static final LoggedTunableNumber intakeVoltage =
            new LoggedTunableNumber("Feeder/IntakeVoltage", 4.0);
    private static final LoggedTunableNumber feedVoltage =
            new LoggedTunableNumber("Feeder/FeedVoltage", 12.0);

    private final FeederIO io;
    private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    private double leftSetpointRpm;
    private double rightSetpointRPM;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    public enum Goal {
        IDLE,
        FEEDING,
        INTAKING,
        REVERSE_INTAKING
    }

    @Getter @Setter private Goal goal = Goal.IDLE;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        if (DriverStation.isDisabled()) {
            stop();
            setGoal(Goal.IDLE);
        } else {
            switch (goal) {
                case IDLE -> runVolts(0.0);
                case FEEDING -> runVolts(feedVoltage.get());
                case INTAKING -> runVolts(intakeVoltage.get());
                case REVERSE_INTAKING -> runVolts(-intakeVoltage.get());
            }
        }

        Logger.recordOutput("Feeder/Goal", goal);
    }

    public boolean hasGamepiece() {
        return inputs.outputCurrent >= 40.0;
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public void stop() {
        io.stop();
    }
}
