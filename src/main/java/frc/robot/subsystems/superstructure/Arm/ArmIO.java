package frc.robot.subsystems.superstructure.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public boolean hasFoc = false;
        public boolean hasAbsoluteSensor = false;
        public double armAnglePositionRads = 0.0;
        public double armTrajectorySetpointRads = 0.0;
        public double armVelocityRadsPerSec = 0.0;
        public double[] armAppliedVolts = new double[] {};
        public double[] armCurrentAmps = new double[] {};
        public double[] armTorqueCurrentAmps = new double[] {};
        public double[] armTempCelcius = new double[] {};
    }

    default void updateInputs(ArmIOInputs inputs) {}

    /** Run to setpoint angle in radians */
    default void setSetpoint(double setpointRads) {}

    /** Run motors at voltage */
    default void setVoltage(double volts) {}

    /** Run motors at current */
    default void setCurrent(double amps) {}

    /** Set brake mode enabled */
    default void setBrakeMode(boolean enabled) {}

    /** Set FF values */
    default void setFF(double s, double v, double a, double g) {}

    /** Set PID values */
    default void setPID(double p, double i, double d) {}

    /** Set MotionMagic constraints */
    default void setProfileConstraints(
            double cruiseVelocityRadsPerSec, double accelerationRadsPerSec2) {}

    default void setPosition(double positionRads) {}

    /** Stops motors */
    default void stop() {}
}
