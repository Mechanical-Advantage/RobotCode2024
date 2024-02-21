// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.junction.Logger;

public class GenericSlamElevator {
  @RequiredArgsConstructor
  public enum Goal {
    RETRACTED(-1),
    EXTENDED(1);

    private final int direction;
  }

  private final GenericSlamElevatorIO io;
  private final GenericSlamElevatorIOInputsAutoLogged inputs =
      new GenericSlamElevatorIOInputsAutoLogged();

  private final String name;
  private final double slammingVolts;
  private final double staticTimeSecs;
  private final double minVelocityThresh;

  private Goal goal;
  private boolean atGoal = false;
  private final Timer staticTimer = new Timer();

  private final Alert disconnected;

  /**
   * Creates a new GenericSlamElevator
   *
   * @param name Name of elevator.
   * @param io IO implementation of elevator.
   * @param slammingVolts Volts to run at when slamming.
   * @param staticTimeSecs Time that it takes for elevator to stop running after hitting the end of
   *     the elevator.
   * @param minVelocityThresh Minimum velocity threshold for elevator to start stopping at in motor
   *     rads/sec.
   */
  public GenericSlamElevator(
      String name,
      GenericSlamElevatorIO io,
      double slammingVolts,
      double staticTimeSecs,
      double minVelocityThresh) {
    this.name = name;
    this.io = io;
    this.slammingVolts = slammingVolts;
    this.staticTimeSecs = staticTimeSecs;
    this.minVelocityThresh = minVelocityThresh;

    disconnected = new Alert(name + " disconnected!", Alert.AlertType.WARNING);
    setGoal(Goal.RETRACTED);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    disconnected.set(!inputs.motorConnected);

    // Run to goal if not at goal.
    if (!atGoal) {
      // Start static timer if within min velocity threshold.
      if (Math.abs(inputs.velocityRadsPerSec) <= minVelocityThresh) {
        staticTimer.start();
      } else {
        staticTimer.reset();
        staticTimer.stop();
      }
      // If we are finished with timer stop and finish goal, otherwise keep on running.
      if (staticTimer.hasElapsed(staticTimeSecs)) {
        io.stop();
        atGoal = true;
      } else {
        io.runVolts(goal.direction * slammingVolts);
        atGoal = false;
        staticTimer.reset();
        staticTimer.stop();
      }
    } else {
      io.stop();
    }

    Logger.recordOutput("Superstructure/" + name + "Goal", goal);
    Logger.recordOutput("Superstructure/" + name + "atGoal", atGoal);
  }

  public void setGoal(Goal goal) {
    if (this.goal == goal) return; // Already set as current goal
    this.goal = goal;
    // Reset state
    atGoal = false;
    staticTimer.reset();
    staticTimer.stop();
  }

  public boolean extended() {
    return goal == Goal.EXTENDED && atGoal;
  }

  public boolean retracted() {
    return goal == Goal.RETRACTED && atGoal;
  }
}
