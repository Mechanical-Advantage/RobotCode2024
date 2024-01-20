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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(100);

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final ModuleIO[] modules = new ModuleIO[4]; // FL, FR, BL, BR
  private final ModuleIOInputsAutoLogged[] moduleInputs =
      new ModuleIOInputsAutoLogged[] {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged()
      };
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  private ChassisSpeeds fieldVelocity = new ChassisSpeeds();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = flModuleIO;
    modules[1] = frModuleIO;
    modules[2] = blModuleIO;
    modules[3] = brModuleIO;
  }

  public void periodic() {
    odometryLock.lock();
    double[] timestamps = timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    // Sim will not have any timstamps
    if (timestamps.length == 0) {
      timestamps = new double[] {Timer.getFPGATimestamp()};
    }
    timestampQueue.clear();
    gyroIO.updateInputs(gyroInputs);
    for (int i = 0; i < modules.length; i++) {
      modules[i].updateInputs(moduleInputs[i]);
    }
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (int i = 0; i < modules.length; i++) {
      Logger.processInputs("Drive/Module" + i, moduleInputs[i]);
      modules[i].periodic();
    }

    // Calculate the min odometry position updates across all modules
    int minOdometryUpdates =
        IntStream.of(
                timestamps.length,
                Arrays.stream(moduleInputs)
                    .mapToInt(
                        moduleInput ->
                            Math.min(
                                moduleInput.odometryDrivePositionsMeters.length,
                                moduleInput.odometryTurnPositions.length))
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
              Arrays.stream(moduleInputs)
                  .map(
                      moduleInput ->
                          new SwerveModulePosition(
                              moduleInput.odometryDrivePositionsMeters[odometryIndex],
                              moduleInput.odometryTurnPositions[odometryIndex]))
                  .toArray(SwerveModulePosition[]::new));
      RobotState.getInstance()
          .addOdometryData(new RobotState.OdometryObservation(wheelPositions, yaw, timestamps[i]));
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // update current velocities
    ChassisSpeeds robotRelativeVelocity =
        DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    robotVelocity = robotRelativeVelocity;
    Translation2d linearFieldVel =
        new Translation2d(
                robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond)
            .rotateBy(getGyroYaw());
    fieldVelocity =
        new ChassisSpeeds(
            linearFieldVel.getX(),
            linearFieldVel.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : robotRelativeVelocity.omegaRadiansPerSecond);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates =
        DriveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.drivetrainConfig.maxLinearVelocity());
    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] =
          SwerveModuleState.optimize(setpointStates[i], modules[i].getAngle());
      modules[i].runSetpoint(optimizedSetpointStates[i]);
    }
    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    DriveConstants.kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  public SwerveDriveWheelPositions getWheelPositions() {
    return new SwerveDriveWheelPositions(
        Arrays.stream(modules).map(ModuleIO::getPosition).toArray(SwerveModulePosition[]::new));
  }

  @AutoLogOutput(key = "Odometry/GyroYaw")
  public Rotation2d getGyroYaw() {
    return gyroInputs.yawPosition;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(ModuleIO::getState).toArray(SwerveModuleState[]::new);
  }

  /** Get current robot relative velocity of robot */
  @AutoLogOutput(key = "Odometry/RobotVelocity")
  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  /** Get current field velocity of robot */
  @AutoLogOutput(key = "Odometry/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return fieldVelocity;
  }
}
