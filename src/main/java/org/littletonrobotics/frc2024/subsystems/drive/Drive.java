// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.AutoAimController;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.AutoAlignController;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.TeleopDriveController;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.TrajectoryController;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.frc2024.util.swerve.ModuleLimits;
import org.littletonrobotics.frc2024.util.swerve.SwerveSetpoint;
import org.littletonrobotics.frc2024.util.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class Drive extends SubsystemBase {
  private static final LoggedTunableNumber coastSpeedLimit =
      new LoggedTunableNumber(
          "Drive/CoastSpeedLimit", DriveConstants.driveConfig.maxLinearVelocity() * 0.6);
  private static final LoggedTunableNumber coastDisableTime =
      new LoggedTunableNumber("Drive/CoastDisableTimeSeconds", 0.5);

  public enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    TELEOP,

    /** Driving based on a trajectory. */
    TRAJECTORY,

    /** Driving to a location on the field automatically. */
    AUTO_ALIGN,

    /** Characterizing (modules oriented forwards, motor outputs supplied externally). */
    CHARACTERIZATION
  }

  @AutoLog
  public static class OdometryTimestampInputs {
    public double[] timestamps = new double[] {};
  }

  public static final Lock odometryLock = new ReentrantLock();
  // TODO: DO THIS BETTER!
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(100);

  private final OdometryTimestampInputsAutoLogged odometryTimestampInputs =
      new OdometryTimestampInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  private SwerveDriveWheelPositions lastPositions = null;
  private double lastTime = 0.0;

  /** Active drive mode. */
  private DriveMode currentDriveMode = DriveMode.TELEOP;

  private double characterizationInput = 0.0;
  private boolean modulesOrienting = false;
  private final Timer coastTimer = new Timer();
  private boolean brakeModeEnabled = true;

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private ModuleLimits currentModuleLimits = DriveConstants.moduleLimits;
  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private SwerveSetpointGenerator setpointGenerator;

  private final TeleopDriveController teleopDriveController;
  private TrajectoryController trajectoryController = null;
  private AutoAlignController autoAlignController = null;
  private AutoAimController autoAimController = null;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);

    setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(DriveConstants.kinematics)
            .moduleLocations(DriveConstants.moduleTranslations)
            .build();
    teleopDriveController = new TeleopDriveController();
  }

  public void periodic() {
    // Update & process inputs
    odometryLock.lock();
    // Read timestamps from odometry thread and fake sim timestamps
    odometryTimestampInputs.timestamps =
        timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    if (odometryTimestampInputs.timestamps.length == 0) {
      odometryTimestampInputs.timestamps = new double[] {Timer.getFPGATimestamp()};
    }
    timestampQueue.clear();
    Logger.processInputs("Drive/OdometryTimestamps", odometryTimestampInputs);
    // Read inputs from gyro
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    // Read inputs from modules
    Arrays.stream(modules).forEach(Module::updateInputs);
    odometryLock.unlock();

    // Calculate the min odometry position updates across all modules
    int minOdometryUpdates =
        IntStream.of(
                odometryTimestampInputs.timestamps.length,
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
      boolean includeMeasurement = true;
      int odometryIndex = i;
      Rotation2d yaw = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;
      // Get all four swerve module positions at that odometry update
      // and store in SwerveDriveWheelPositions object
      SwerveDriveWheelPositions wheelPositions =
          new SwerveDriveWheelPositions(
              Arrays.stream(modules)
                  .map(module -> module.getModulePositions()[odometryIndex])
                  .toArray(SwerveModulePosition[]::new));
      // Filtering
      if (lastPositions != null) {
        double dt = Timer.getFPGATimestamp() - lastTime;
        for (int j = 0; j < 4; j++) {
          double velocity =
              (wheelPositions.positions[j].distanceMeters
                      - lastPositions.positions[j].distanceMeters)
                  / dt;
          double omega =
              wheelPositions.positions[j].angle.minus(lastPositions.positions[j].angle).getRadians()
                  / dt;

          if (Math.abs(omega) > currentModuleLimits.maxSteeringVelocity() * 100.0
              || Math.abs(velocity) > currentModuleLimits.maxDriveVelocity() * 100.0) {
            includeMeasurement = false;
            break;
          }
        }
      }
      if (includeMeasurement) {
        lastPositions = wheelPositions;
        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    wheelPositions, yaw, odometryTimestampInputs.timestamps[i]));
      }
    }
    lastTime = Timer.getFPGATimestamp();

    // update current velocities use gyro when possible
    ChassisSpeeds robotRelativeVelocity = getSpeeds();
    robotRelativeVelocity.omegaRadiansPerSecond =
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : robotRelativeVelocity.omegaRadiansPerSecond;
    RobotState.getInstance().addVelocityData(robotRelativeVelocity.toTwist2d());

    // Disabled, stop modules and coast
    if (DriverStation.isDisabled()) {
      Arrays.stream(modules).forEach(Module::stop);
      if (Math.hypot(
                  robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond)
              <= coastSpeedLimit.get()
          && brakeModeEnabled) {
        setBrakeMode(false);
        coastTimer.stop();
        coastTimer.reset();
      } else if (coastTimer.hasElapsed(coastDisableTime.get()) && brakeModeEnabled) {
        setBrakeMode(false);
        coastTimer.stop();
        coastTimer.reset();
      } else {
        coastTimer.start();
      }
      return;
    } else {
      // Brake mode
      setBrakeMode(true);
    }

    // Run drive based on current mode
    ChassisSpeeds teleopSpeeds = teleopDriveController.update();
    switch (currentDriveMode) {
      case TELEOP -> {
        // Plain teleop drive
        desiredSpeeds = teleopSpeeds;
        // Add auto aim if present
        if (autoAimController != null) {
          desiredSpeeds.omegaRadiansPerSecond = autoAimController.update();
        }
      }
      case TRAJECTORY -> {
        // Run trajectory
        desiredSpeeds = trajectoryController.update();
      }
      case AUTO_ALIGN -> {
        // Run auto align with drive input
        desiredSpeeds = autoAlignController.update();
        desiredSpeeds.vxMetersPerSecond += teleopSpeeds.vxMetersPerSecond * 0.1;
        desiredSpeeds.vyMetersPerSecond += teleopSpeeds.vyMetersPerSecond * 0.1;
        desiredSpeeds.omegaRadiansPerSecond += teleopSpeeds.omegaRadiansPerSecond * 0.1;
      }
      case CHARACTERIZATION -> {
        // run characterization
        for (Module module : modules) {
          module.runCharacterization(characterizationInput);
        }
      }
      default -> {}
    }

    // Run robot at desiredSpeeds
    // Generate feasible next setpoint
    currentSetpoint =
        setpointGenerator.generateSetpoint(
            currentModuleLimits, currentSetpoint, desiredSpeeds, 0.02);
    // run modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      // Optimize setpoints
      optimizedSetpointStates[i] =
          SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());
      modules[i].runSetpoint(optimizedSetpointStates[i]);
    }
    // Log chassis speeds and swerve states
    Logger.recordOutput(
        "Drive/SwerveStates/Desired(b4 Poofs)",
        DriveConstants.kinematics.toSwerveModuleStates(desiredSpeeds));
    Logger.recordOutput("Drive/SwerveStates/Setpoints", optimizedSetpointStates);
    Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
    Logger.recordOutput("Drive/SetpointSpeeds", currentSetpoint.chassisSpeeds());
    Logger.recordOutput("Drive/DriveMode", currentDriveMode);
  }

  /** Pass controller input into teleopDriveController in field relative input */
  public void setTeleopDriveGoal(double controllerX, double controllerY, double controllerOmega) {
    if (DriverStation.isTeleopEnabled()) {
      if (currentDriveMode != DriveMode.AUTO_ALIGN) {
        currentDriveMode = DriveMode.TELEOP;
      }
      teleopDriveController.acceptDriveInput(controllerX, controllerY, controllerOmega);
    }
  }

  /** Sets the trajectory for the robot to follow. */
  public void setTrajectoryGoal(HolonomicTrajectory trajectory) {
    if (DriverStation.isAutonomousEnabled()) {
      currentDriveMode = DriveMode.TRAJECTORY;
      trajectoryController = new TrajectoryController(trajectory);
    }
  }

  /** Clears the current trajectory goal. */
  public void clearTrajectoryGoal() {
    trajectoryController = null;
    currentDriveMode = DriveMode.TELEOP;
  }

  /** Returns true if the robot is done with trajectory. */
  @AutoLogOutput(key = "Drive/TrajectoryCompleted")
  public boolean isTrajectoryGoalCompleted() {
    return trajectoryController != null && trajectoryController.isFinished();
  }

  /** Sets the goal pose for the robot to drive to */
  public void setAutoAlignGoal(Pose2d goalPose) {
    if (DriverStation.isTeleopEnabled()) {
      currentDriveMode = DriveMode.AUTO_ALIGN;
      autoAlignController = new AutoAlignController(goalPose);
    }
  }

  /** Clears the current auto align goal. */
  public void clearAutoAlignGoal() {
    autoAlignController = null;
    currentDriveMode = DriveMode.TELEOP;
  }

  /** Returns true if the robot is at current goal pose. */
  @AutoLogOutput(key = "Drive/AutoAlignCompleted")
  public boolean isAutoAlignGoalCompleted() {
    return autoAlignController != null && autoAlignController.atGoal();
  }

  /** Enable auto aiming on drive */
  public void setAutoAimGoal() {
    autoAimController = new AutoAimController();
  }

  /** Disable auto aiming on drive */
  public void clearAutoAimGoal() {
    autoAimController = null;
  }

  /** Returns true if robot is aimed at speaker */
  @AutoLogOutput(key = "Drive/AutoAimCompleted")
  public boolean isAutoAimGoalCompleted() {
    return autoAimController != null && autoAimController.atSetpoint();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    currentDriveMode = DriveMode.CHARACTERIZATION;
    characterizationInput = volts;
  }

  /** Disables the characterization mode. */
  public void endCharacterization() {
    currentDriveMode = DriveMode.TELEOP;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Set brake mode enabled */
  public void setBrakeMode(boolean enabled) {
    brakeModeEnabled = enabled;
    Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
  }

  public Command orientModules(Rotation2d orientation) {
    return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
  }

  public Command orientModules(Rotation2d[] orientations) {
    return run(() -> {
          for (int i = 0; i < orientations.length; i++) {
            modules[i].runSetpoint(new SwerveModuleState(0.0, orientations[i]));
          }
        })
        .until(
            () ->
                Arrays.stream(modules)
                    .allMatch(
                        module ->
                            Math.abs(
                                    module.getAngle().getDegrees()
                                        - module.getSetpointState().angle.getDegrees())
                                <= 2.0))
        .beforeStarting(() -> modulesOrienting = true)
        .finallyDo(() -> modulesOrienting = false);
  }

  @AutoLogOutput(key = "Drive/GyroYaw")
  public Rotation2d getGyroYaw() {
    return gyroInputs.connected
        ? gyroInputs.yawPosition
        : RobotState.getInstance().getEstimatedPose().getRotation();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  /** Returns the measured speeds of the robot in the robot's frame of reference. */
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
  private ChassisSpeeds getSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  public static Rotation2d[] getStraightOrientations() {
    return IntStream.range(0, 4).mapToObj(Rotation2d::new).toArray(Rotation2d[]::new);
  }

  public static Rotation2d[] getXOrientations() {
    return Arrays.stream(DriveConstants.moduleTranslations)
        .map(Translation2d::getAngle)
        .toArray(Rotation2d[]::new);
  }
}
