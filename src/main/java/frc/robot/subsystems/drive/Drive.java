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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final Lock odometryLock = new ReentrantLock();
  // TODO: DO THIS BETTER!
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(100);

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final DriveMotionPlanner motionPlanner = new DriveMotionPlanner();
  private Twist2d robotVelocity = new Twist2d();
  private Twist2d fieldVelocity = new Twist2d();

  private boolean characterizing = false;
  private double characterizationVolts = 0.0;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);
  }

  public void periodic() {
    // Update & process inputs
    odometryLock.lock();
    // Read timestamps from odometry thread
    double[] timestamps = timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    // fake sim timestamps
    if (timestamps.length == 0) {
      timestamps = new double[] {Timer.getFPGATimestamp()};
    }
    timestampQueue.clear();
    // Read inputs from gyro and modules
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Arrays.stream(modules).forEach(Module::updateInputs);
    odometryLock.unlock();

    // Call periodic on module
    Arrays.stream(modules).forEach(Module::periodic);

    // Calculate the min odometry position updates across all modules
    int minOdometryUpdates =
        IntStream.of(
                timestamps.length,
                Arrays.stream(modules)
                    .mapToInt(module -> module.getModulePositions().length)
                    .min()
                    .orElse(0))
            .min()
            .orElse(0);
    if (gyroInputs.connected) {
      minOdometryUpdates = Math.min(gyroInputs.odometryYawPositions.length, minOdometryUpdates);
    }
    // Pass odometry data to robot state
    for (int i = 0; i < minOdometryUpdates; i++) {
      int odometryIndex = i;
      Rotation2d yaw = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;
      // Get all four swerve module positions at that odometry update
      // and store in SwerveDriveWheelPositions object
      SwerveDriveWheelPositions wheelPositions =
          new SwerveDriveWheelPositions(
              Arrays.stream(modules)
                  .map(module -> module.getModulePositions()[odometryIndex])
                  .toArray(SwerveModulePosition[]::new));
      RobotState.getInstance()
          .addOdometryObservation(
              new RobotState.OdometryObservation(wheelPositions, yaw, timestamps[i]));
    }

    // update current velocities use gyro when possible
    ChassisSpeeds robotRelativeVelocity =
        DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    robotRelativeVelocity.omegaRadiansPerSecond =
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : robotRelativeVelocity.omegaRadiansPerSecond;
    robotVelocity =
        new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond,
            robotRelativeVelocity.vyMetersPerSecond,
            robotRelativeVelocity.omegaRadiansPerSecond);
    Translation2d linearFieldVel =
        new Translation2d(
                robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond)
            .rotateBy(getGyroYaw());
    fieldVelocity = new Twist2d(linearFieldVel.getX(), linearFieldVel.getY(), robotVelocity.dtheta);

    // Add drive data
    RobotState.getInstance().addDriveData(getWheelPositions(), getGyroYaw(), fieldVelocity);

    // Get motion planner
    SwerveModuleState[] setpointStates =
        motionPlanner.update(
            Timer.getFPGATimestamp(), RobotState.getInstance().getEstimatedPose(), fieldVelocity);
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    // Run robot
    if (DriverStation.isDisabled()) {
      // Log empty setpoint
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    } else if (DriverStation.isEnabled()) {
      // Optimize setpoints
      for (int i = 0; i < modules.length; i++) {
        optimizedSetpointStates[i] =
            SwerveModuleState.optimize(setpointStates[i], modules[i].getAngle());
        // Run modules
        modules[i].runSetpoint(optimizedSetpointStates[i]);
        // Log setpoint states
      }
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    } else if (characterizing) {
      // Run characterization
      for (Module module : modules) {
        module.runCharacterization(characterizationVolts);
      }
    }
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    characterizing = true;
    characterizationVolts = volts;
  }

  public void endCharacterization() {
    characterizing = false;
  }

  public DriveMotionPlanner getMotionPlanner() {
    return motionPlanner;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  private SwerveDriveWheelPositions getWheelPositions() {
    return new SwerveDriveWheelPositions(
        Arrays.stream(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new));
  }

  @AutoLogOutput(key = "Odometry/GyroYaw")
  public Rotation2d getGyroYaw() {
    return gyroInputs.connected
        ? gyroInputs.yawPosition
        : RobotState.getInstance().getEstimatedPose().getRotation();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  /** Get current robot relative velocity of robot */
  @AutoLogOutput(key = "Odometry/RobotVelocity")
  public Twist2d getRobotVelocity() {
    return robotVelocity;
  }

  /** Get current field velocity of robot */
  @AutoLogOutput(key = "Odometry/FieldVelocity")
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  public static Rotation2d[] getXOrientations() {
    return Arrays.stream(DriveConstants.moduleTranslations)
        .map(Translation2d::getAngle)
        .toArray(Rotation2d[]::new);
  }
}
