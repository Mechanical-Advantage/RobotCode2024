// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double armPositionRads = 0.0;
    public double armEncoderPositionRads = 0.0;
    public double armAbsoluteEncoderPositionRads = 0.0;
    public double armTrajectorySetpointRads = 0.0;
    public double armVelocityRadsPerSec = 0.0;
    public double[] armAppliedVolts = new double[] {};
    public double[] armCurrentAmps = new double[] {};
    public double[] armTorqueCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};
    public boolean absoluteEncoderConnected = true;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  default void runSetpoint(double setpointRads) {}

  /** Run motors at volts */
  default void runVolts(double volts) {}

  /** Run motors at current */
  default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  default void setBrakeMode(boolean enabled) {}

  /** Set FF values */
  default void setFF(double s, double v, double a, double g) {}

  /** Set PID values */
  default void setPID(double p, double i, double d) {}

  /** Set MotionMagic constraints */
  default void setProfileConstraints(
      double cruiseVelocityRadsPerSec, double accelerationRadsPerSec2) {}

  /** Sets position of internal encoder */
  default void setPosition(double positionRads) {}

  /** Stops motors */
  default void stop() {}
}
