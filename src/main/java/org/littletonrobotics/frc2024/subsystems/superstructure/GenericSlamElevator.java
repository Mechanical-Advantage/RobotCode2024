// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public abstract class GenericSlamElevator<G extends GenericSlamElevator.SlamElevatorGoal> {

  public interface SlamElevatorGoal {
    /** Returns either -1 or 1 based on directino of slamming */
    int getDirection();

    boolean isStopAtGoal();
  }

  private final GenericSlamElevatorIO io;
  private final GenericSlamElevatorIOInputsAutoLogged inputs =
      new GenericSlamElevatorIOInputsAutoLogged();

  private final String name;
  private final double slammingCurrent;
  private final double staticTimeSecs;
  private final double minVelocityThresh;

  protected abstract G getGoal();

  private G currentGoal = null;
  private G lastGoal = null;

  private boolean atGoal = false;
  private final Timer staticTimer = new Timer();

  private boolean brakeModeEnabled = false;
  private BooleanSupplier coastModeSupplier = () -> false;

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
    setBrakeMode(true);

    disconnected = new Alert(name + " disconnected!", Alert.AlertType.WARNING);
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    coastModeSupplier = coastOverride;
  }

  private void setBrakeMode(boolean enable) {
    if (brakeModeEnabled == enable) return;
    brakeModeEnabled = enable;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Ensure brake mode is enabled
    if (DriverStation.isEnabled()) {
      setBrakeMode(true);
    }

    currentGoal = getGoal();
    // Reset if changing goals
    if (lastGoal != null && currentGoal != lastGoal) {
      atGoal = false;
      staticTimer.stop();
      staticTimer.reset();
    }

    // Set alert
    disconnected.set(!inputs.motorConnected);

    // Check if at goal.
    if (!atGoal) {
      // Start static timer if within min velocity threshold.
      if (Math.abs(inputs.velocityRadsPerSec) <= minVelocityThresh) {
        staticTimer.start();
      } else {
        staticTimer.stop();
        staticTimer.reset();
      }
      // If we are finished with timer, finish goal.
      atGoal = staticTimer.hasElapsed(staticTimeSecs);
    } else {
      staticTimer.stop();
      staticTimer.reset();
    }

    // Run to goal.
    if (!atGoal) {
      io.runCurrent(currentGoal.getDirection() * slammingCurrent);
    } else {
      if (currentGoal.isStopAtGoal()) {
        io.stop();
      } else {
        io.runCurrent(currentGoal.getDirection() * slammingCurrent);
      }
    }

    // Set last goal
    lastGoal = currentGoal;

    if (DriverStation.isDisabled()) {
      // Reset
      io.stop();
      currentGoal = null;
      lastGoal = null;
      atGoal = false;
      staticTimer.stop();
      staticTimer.reset();
      // Set to coast mode
      setBrakeMode(coastModeSupplier.getAsBoolean());
    }

    Logger.recordOutput("Superstructure/" + name + "/Goal", getGoal().toString());
    Logger.recordOutput("Superstructure/" + name + "/atGoal", atGoal);
    Logger.recordOutput("Superstructure/" + name + "/Extended", extended());
    Logger.recordOutput("Superstructure/" + name + "/Retracted", retracted());
  }

  public boolean atGoal() {
    return atGoal;
  }

  public boolean extended() {
    return getGoal().getDirection() == 1 && atGoal;
  }

  public boolean retracted() {
    return getGoal().getDirection() == -1 && atGoal;
  }
}
