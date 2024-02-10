package org.littletonrobotics.frc2024.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final LoggedTunableNumber intakeVolts =
            new LoggedTunableNumber("Intake/intakeVoltage", 8.0);
    private final LoggedTunableNumber ejectVolts = new LoggedTunableNumber("Intake/ejectVolts", -2.0);
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum Goal {
        IDLE,
        INTAKE,
        REVERSE_INTAKE
    }

    @Setter @Getter Goal goal = Goal.IDLE;

    public Intake(IntakeIO io) {
        System.out.println("[Init] Creating Intake");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        switch (goal) {
            case IDLE -> io.stop();
            case INTAKE -> io.runVolts(intakeVolts.get());
            case REVERSE_INTAKE -> io.runVolts(ejectVolts.get());
        }
    }
}
