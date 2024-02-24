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
    public boolean driveMotorConnected = true;
    public boolean turnMotorConnected = true;
    public boolean hasCurrentControl = false;

    public double drivePositionRads = 0.0;
    public double driveVelocityRadsPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadsPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;

    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run drive motor at volts */
  default void runDriveVolts(double volts) {}

  /** Run turn motor at volts */
  default void runTurnVolts(double volts) {}

  /** Run drive motor at current */
  default void runDriveCurrent(double current) {}

  /** Run turn motor at current */
  default void runTurnCurrent(double current) {}

  /** Run to drive velocity setpoint with feedforward */
  default void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {}

  /** Run to turn position setpoint */
  default void runTurnPositionSetpoint(double angleRads) {}

  /** Configure drive PID */
  default void setDrivePID(double kP, double kI, double kD) {}

  /** Configure turn PID */
  default void setTurnPID(double kP, double kI, double kD) {}

  /** Enable or disable brake mode on the drive motor. */
  default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  default void setTurnBrakeMode(boolean enable) {}

  /** Disable output to all motors */
  default void stop() {}
}
