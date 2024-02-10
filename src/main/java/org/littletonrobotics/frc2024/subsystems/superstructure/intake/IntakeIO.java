package org.littletonrobotics.frc2024.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double velocityRadsPerSec = 0.0;
        public double positionRads = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
    }

    /** Update inputs */
    default void updateInputs(IntakeIOInputs inputs) {}

    /** Set voltage of intake */
    default void runVolts(double volts) {}

    /** Stop intake */
    default void stop() {}
}
