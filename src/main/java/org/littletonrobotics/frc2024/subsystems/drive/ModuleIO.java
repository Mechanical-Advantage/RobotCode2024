// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;

    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run drive motor at volts */
  default void setDriveVoltage(double volts) {}

  /** Run turn motor at volts */
  default void setTurnVoltage(double volts) {}

  /** Set drive velocity setpoint */
  default void setDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {}

  /** Set turn position setpoint */
  default void setTurnPositionSetpoint(double angleRads) {}

  /** Configure drive PID */
  default void setDrivePID(double kP, double kI, double kD) {}

  /** Configure turn PID */
  default void setTurnPID(double kP, double kI, double kD) {}

  /** Configure drive feedforward for TorqueCurrentFOC */
  default void setDriveFF(double kS, double kV, double kA) {}

  /** Enable or disable brake mode on the drive motor. */
  default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  default void setTurnBrakeMode(boolean enable) {}

  /** Disable output to all motors */
  default void stop() {}
}
