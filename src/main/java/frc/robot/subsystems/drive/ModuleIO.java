// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryDrivePositionsMeters = new double[]{};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ModuleIOInputs inputs) {
    }

    /**
     * Run drive motor at volts
     */
    default void setDriveVoltage(double volts) {
    }

    /**
     * Run turn motor at volts
     */
    default void setTurnVoltage(double volts) {
    }

    /**
     * Set drive velocity setpoint
     */
    default void setDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
    }

    /**
     * Set turn position setpoint
     */
    default void setTurnPositionSetpoint(double angleRads) {
    }

    /**
     * Configure drive PID
     */
    default void setDrivePID(double kP, double kI, double kD) {
    }

    /**
     * Configure turn PID
     */
    default void setTurnPID(double kP, double kI, double kD) {
    }

    /**
     * Configure drive feedforward for TorqueCurrentFOC
     */
    default void setDriveFF(double kS, double kV, double kA) {
    }

    /**
     * Enable or disable brake mode on the drive motor.
     */
    default void setDriveBrakeMode(boolean enable) {
    }

    /**
     * Enable or disable brake mode on the turn motor.
     */
    default void setTurnBrakeMode(boolean enable) {
    }

    /**
     * Disable output to all motors
     */
    default void stop() {
    }
}
