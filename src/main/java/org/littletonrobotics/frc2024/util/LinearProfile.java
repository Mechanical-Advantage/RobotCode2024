// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.util;

import lombok.Getter;
import lombok.Setter;

/** Ramps up and down to setpoint for velocity closed loop control */
public class LinearProfile {
  private double dv;
  @Getter private final double period;
  @Getter private double currentSetpoint = 0;
  @Getter @Setter private double goal = 0;

  /**
   * Creates a new LinearProfiler
   *
   * @param maxAcceleration The max ramp rate in velocity in rpm/sec
   * @param period Period of control loop (0.02)
   */
  public LinearProfile(double maxAcceleration, double period) {
    this.period = period;
    setMaxAcceleration(maxAcceleration);
  }

  /** Set the max acceleration constraint in rpm/sec */
  public void setMaxAcceleration(double maxAcceleration) {
    dv = maxAcceleration * period;
  }

  /**
   * Sets the target setpoint, starting from the current speed
   *
   * @param goal Target setpoint
   * @param currentSpeed Current speed, to be used as the starting setpoint
   */
  public void setGoal(double goal, double currentSpeed) {
    this.goal = goal;
    currentSetpoint = currentSpeed;
  }

  /** Resets target setpoint and current setpoint */
  public void reset() {
    currentSetpoint = 0;
    goal = 0;
  }

  /**
   * Returns the current setpoint to send to motors
   *
   * @return Setpoint to send to motors
   */
  public double calculateSetpoint() {
    if (goal > currentSetpoint) {
      currentSetpoint += dv;
      if (currentSetpoint > goal) {
        currentSetpoint = goal;
      }
    } else if (goal < currentSetpoint) {
      currentSetpoint -= dv;
      if (currentSetpoint < goal) {
        currentSetpoint = goal;
      }
    }
    return currentSetpoint;
  }
}
