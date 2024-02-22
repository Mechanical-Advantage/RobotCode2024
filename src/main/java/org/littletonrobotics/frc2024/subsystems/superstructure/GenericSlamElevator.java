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
    IDLE(0),
    RETRACT(-1),
    EXTEND(1);

    private final int value;
  }

  private final GenericSlamElevatorIO io;
  private final GenericSlamElevatorIOInputsAutoLogged inputs =
      new GenericSlamElevatorIOInputsAutoLogged();

  private final String name;
  private final double slammingCurrent;
  private final double staticTimeSecs;
  private final double minVelocityThresh;

  private Goal goal = Goal.RETRACT;
  private boolean atGoal = false;
  private final Timer staticTimer = new Timer();

  private final Alert disconnected;

  /**
   * Creates a new GenericSlamElevator
   *
   * @param name Name of elevator.
   * @param io IO implementation of elevator.
   * @param slammingCurrent Amps to run at when slamming.
   * @param staticTimeSecs Time that it takes for elevator to stop running after hitting the end of
   *     the elevator.
   * @param minVelocityThresh Minimum velocity threshold for elevator to start stopping at in motor
   *     rads/sec.
   */
  public GenericSlamElevator(
      String name,
      GenericSlamElevatorIO io,
      double slammingCurrent,
      double staticTimeSecs,
      double minVelocityThresh) {
    this.name = name;
    this.io = io;
    this.slammingCurrent = slammingCurrent;
    this.staticTimeSecs = staticTimeSecs;
    this.minVelocityThresh = minVelocityThresh;

    disconnected = new Alert(name + " disconnected!", Alert.AlertType.WARNING);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Set alert
    disconnected.set(!inputs.motorConnected);

    // Run the io at goal
    io.runCurrent(goal.value * slammingCurrent);

    // Check if at goal.
    if (!atGoal) {
      // Start static timer if within min velocity threshold.
      if (Math.abs(inputs.velocityRadsPerSec) <= minVelocityThresh) {
        staticTimer.start();
      } else {
        staticTimer.reset();
        staticTimer.stop();
      }
      // If we are finished with timer stop and finish goal, otherwise keep on running.
      atGoal = staticTimer.hasElapsed(staticTimeSecs);
    } else {
      staticTimer.stop();
      staticTimer.reset();
    }

    Logger.recordOutput(name + "/Goal", goal);
    Logger.recordOutput(name + "/atGoal", atGoal);
  }

  public void setGoal(Goal goal) {
    if (this.goal == goal) return; // Already set as current goal
    this.goal = goal;
    // Reset state
    atGoal = false;
    staticTimer.reset();
    staticTimer.stop();
  }

  public boolean atGoal() {
    return atGoal;
  }

  public boolean extended() {
    return goal == Goal.EXTEND && atGoal;
  }

  public boolean retracted() {
    return goal == Goal.RETRACT && atGoal;
  }
}
