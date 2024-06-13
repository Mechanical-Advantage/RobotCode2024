// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.AutoAlignController;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.HeadingController;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.TeleopDriveController;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.TrajectoryController;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.leds.Leds;
import org.littletonrobotics.frc2024.util.EqualsUtil;
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
  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecThreshold =
      new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", 0.05);

  public enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    TELEOP,

    /** Driving based on a trajectory. */
    TRAJECTORY,

    /** Driving to a location on the field automatically. */
    AUTO_ALIGN,

    /** Characterizing (modules oriented forwards, motor outputs supplied externally). */
    CHARACTERIZATION,

    /** Running wheel radius characterization routine (spinning in circle) */
    WHEEL_RADIUS_CHARACTERIZATION
  }

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }

  @AutoLog
  public static class OdometryTimestampInputs {
    public double[] timestamps = new double[] {};
  }

  public static final Lock odometryLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);

  private final OdometryTimestampInputsAutoLogged odometryTimestampInputs =
      new OdometryTimestampInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  // Store previous positions and time for filtering odometry data
  private SwerveDriveWheelPositions lastPositions = null;
  private double lastTime = 0.0;

  /** Active drive mode. */
  private DriveMode currentDriveMode = DriveMode.TELEOP;

  private double characterizationInput = 0.0;
  private boolean modulesOrienting = false;
  private final Timer lastMovementTimer = new Timer();

  @AutoLogOutput(key = "Drive/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  @Setter
  @AutoLogOutput(key = "Drive/CoastRequest")
  private CoastRequest coastRequest = CoastRequest.AUTOMATIC;

  private boolean lastEnabled = false;

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private final SwerveSetpointGenerator setpointGenerator;

  private final TeleopDriveController teleopDriveController;
  private TrajectoryController trajectoryController = null;
  private AutoAlignController autoAlignController = null;
  private HeadingController headingController = null;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);
    lastMovementTimer.start();
    setBrakeMode(true);

    setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(DriveConstants.kinematics)
            .moduleLocations(DriveConstants.moduleTranslations)
            .build();
    teleopDriveController = new TeleopDriveController();

    // Preload trajectory classes
    for (int i = 0; i < 25; i++) {
      var controller = new TrajectoryController(new HolonomicTrajectory("driveStraight"));
      for (int x = 0; x < 25; x++) {
        controller.update();
      }
    }
    RobotState.getInstance().addTrajectoryVelocityData(new Twist2d());
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

    ModuleLimits currentModuleLimits = RobotState.getInstance().getModuleLimits();

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
      int odometryIndex = i;
      Rotation2d yaw = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;
      // Get all four swerve module positions at that odometry update
      // and store in SwerveDriveWheelPositions object
      SwerveDriveWheelPositions wheelPositions =
          new SwerveDriveWheelPositions(
              Arrays.stream(modules)
                  .map(module -> module.getModulePositions()[odometryIndex])
                  .toArray(SwerveModulePosition[]::new));
      // Filtering based on delta wheel positions
      boolean includeMeasurement = true;
      if (lastPositions != null) {
        double dt = odometryTimestampInputs.timestamps[i] - lastTime;
        for (int j = 0; j < modules.length; j++) {
          double velocity =
              (wheelPositions.positions[j].distanceMeters
                      - lastPositions.positions[j].distanceMeters)
                  / dt;
          double omega =
              wheelPositions.positions[j].angle.minus(lastPositions.positions[j].angle).getRadians()
                  / dt;
          // Check if delta is too large
          if (Math.abs(omega) > currentModuleLimits.maxSteeringVelocity() * 5.0
              || Math.abs(velocity) > currentModuleLimits.maxDriveVelocity() * 5.0) {
            includeMeasurement = false;
            break;
          }
        }
      }
      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastPositions = wheelPositions;
        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    wheelPositions, yaw, odometryTimestampInputs.timestamps[i]));
        lastTime = odometryTimestampInputs.timestamps[i];
      }
    }

    // Update current velocities use gyro when possible
    ChassisSpeeds robotRelativeVelocity = getSpeeds();
    robotRelativeVelocity.omegaRadiansPerSecond =
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : robotRelativeVelocity.omegaRadiansPerSecond;
    RobotState.getInstance().addVelocityData(robotRelativeVelocity.toTwist2d());

    // Update brake mode
    // Reset movement timer if moved
    if (Arrays.stream(modules)
        .anyMatch(
            module ->
                Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecThreshold.get())) {
      lastMovementTimer.reset();
    }
    if (DriverStation.isEnabled() && !lastEnabled) {
      coastRequest = CoastRequest.AUTOMATIC;
    }

    lastEnabled = DriverStation.isEnabled();
    switch (coastRequest) {
      case AUTOMATIC -> {
        if (DriverStation.isEnabled()) {
          setBrakeMode(true);
        } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
          setBrakeMode(false);
        }
      }
      case ALWAYS_BRAKE -> {
        setBrakeMode(true);
      }
      case ALWAYS_COAST -> {
        setBrakeMode(false);
      }
    }

    // Run drive based on current mode
    ChassisSpeeds teleopSpeeds = teleopDriveController.update();
    Leds.getInstance().autoDrive = false;
    switch (currentDriveMode) {
      case TELEOP -> {
        // Plain teleop drive
        desiredSpeeds = teleopSpeeds;
        // Add auto aim if present
        if (headingController != null) {
          desiredSpeeds.omegaRadiansPerSecond = headingController.update();
        }
      }
      case TRAJECTORY -> {
        // Run trajectory
        desiredSpeeds = trajectoryController.update();
        if (headingController != null) {
          desiredSpeeds.omegaRadiansPerSecond = headingController.update();
        }
      }
      case AUTO_ALIGN -> {
        // Run auto align with drive input
        desiredSpeeds = autoAlignController.update();
        Leds.getInstance().autoDrive = true;
      }
      case CHARACTERIZATION -> {
        // Run characterization
        for (Module module : modules) {
          module.runCharacterization(0.0, characterizationInput);
        }
      }
      case WHEEL_RADIUS_CHARACTERIZATION -> {
        desiredSpeeds = new ChassisSpeeds(0, 0, characterizationInput);
      }
      default -> {}
    }

    // Run modules
    if (currentDriveMode != DriveMode.CHARACTERIZATION && !modulesOrienting) {
      // Run robot at desiredSpeeds
      // Generate feasible next setpoint
      currentSetpoint =
          setpointGenerator.generateSetpoint(
              currentModuleLimits, currentSetpoint, desiredSpeeds, Constants.loopPeriodSecs);
      SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
      SwerveModuleState[] optimizedSetpointTorques = new SwerveModuleState[4];

      for (int i = 0; i < modules.length; i++) {
        // Optimize setpoints
        optimizedSetpointStates[i] =
            SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());

        if (currentDriveMode == DriveMode.TRAJECTORY && trajectoryController != null) {
          // Only do torque FF in trajectory mode
          Vector<N2> wheelDirection =
              VecBuilder.fill(
                  optimizedSetpointStates[i].angle.getCos(),
                  optimizedSetpointStates[i].angle.getSin());
          Vector<N2> wheelForces = trajectoryController.getModuleForces().get(i);
          double wheelTorque =
              wheelForces.dot(wheelDirection) * DriveConstants.driveConfig.wheelRadius();
          optimizedSetpointTorques[i] =
              new SwerveModuleState(wheelTorque, optimizedSetpointStates[i].angle);
        } else {
          optimizedSetpointTorques[i] =
              new SwerveModuleState(0.0, optimizedSetpointStates[i].angle);
        }
        modules[i].runSetpoint(optimizedSetpointStates[i], optimizedSetpointTorques[i]);
      }
      Logger.recordOutput("Drive/SwerveStates/Setpoints", optimizedSetpointStates);
      Logger.recordOutput("Drive/SwerveStates/Torques", optimizedSetpointTorques);
    }

    // Log chassis speeds and swerve states
    Logger.recordOutput(
        "Drive/SwerveStates/Desired(b4 Poofs)",
        DriveConstants.kinematics.toSwerveModuleStates(desiredSpeeds));
    Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
    Logger.recordOutput("Drive/SetpointSpeeds", currentSetpoint.chassisSpeeds());
    Logger.recordOutput("Drive/DriveMode", currentDriveMode);
  }

  /** Pass controller input into teleopDriveController in field relative input */
  public void acceptTeleopInput(
      double controllerX, double controllerY, double controllerOmega, boolean robotRelative) {
    if (DriverStation.isTeleopEnabled()) {
      if (currentDriveMode != DriveMode.AUTO_ALIGN) {
        currentDriveMode = DriveMode.TELEOP;
      }
      teleopDriveController.acceptDriveInput(
          controllerX, controllerY, controllerOmega, robotRelative);
    }
  }

  /** Sets the trajectory for the robot to follow. */
  public void setTrajectory(HolonomicTrajectory trajectory) {
    if (DriverStation.isAutonomousEnabled()) {
      currentDriveMode = DriveMode.TRAJECTORY;
      trajectoryController = new TrajectoryController(trajectory);
    }
  }

  /** Clears the current trajectory goal. */
  public void clearTrajectory() {
    trajectoryController = null;
    currentDriveMode = DriveMode.TELEOP;
    RobotState.getInstance().addTrajectoryVelocityData(new Twist2d());
  }

  /** Returns true if the robot is done with trajectory. */
  @AutoLogOutput(key = "Drive/TrajectoryCompleted")
  public boolean isTrajectoryCompleted() {
    return trajectoryController != null && trajectoryController.isFinished();
  }

  /** Sets the goal pose for the robot to drive to */
  public void setAutoAlignGoal(
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> feedforwardSupplier,
      boolean slowMode) {
    if (DriverStation.isEnabled()) {
      currentDriveMode = DriveMode.AUTO_ALIGN;
      autoAlignController = new AutoAlignController(poseSupplier, feedforwardSupplier, slowMode);
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
    return autoAlignController == null || autoAlignController.atGoal();
  }

  /** Enable auto aiming on drive */
  public void setHeadingGoal(Supplier<Rotation2d> goalHeadingSupplier) {
    headingController = new HeadingController(goalHeadingSupplier);
  }

  /** Disable auto aiming on drive */
  public void clearHeadingGoal() {
    headingController = null;
  }

  /** Returns true if robot is aimed at speaker */
  @AutoLogOutput(key = "Drive/AtHeadingGoal")
  public boolean atHeadingGoal() {
    return headingController == null || headingController.atGoal();
  }

  /** Runs forwards at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    currentDriveMode = DriveMode.CHARACTERIZATION;
    characterizationInput = input;
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

  /** Runs in a circle at omega. */
  public void runWheelRadiusCharacterization(double omegaSpeed) {
    currentDriveMode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
    characterizationInput = omegaSpeed;
  }

  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRads).toArray();
  }

  /** Set brake mode to {@code enabled} doesn't change brake mode if already set. */
  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  /**
   * Returns command that orients all modules to {@code orientation}, ending when the modules have
   * rotated.
   */
  public Command orientModules(Rotation2d orientation) {
    return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
  }

  /**
   * Returns command that orients all modules to {@code orientations[]}, ending when the modules
   * have rotated.
   */
  public Command orientModules(Rotation2d[] orientations) {
    return run(() -> {
          SwerveModuleState[] states = new SwerveModuleState[4];
          for (int i = 0; i < orientations.length; i++) {
            modules[i].runSetpoint(
                new SwerveModuleState(0.0, orientations[i]),
                new SwerveModuleState(0.0, new Rotation2d()));
            states[i] = new SwerveModuleState(0.0, modules[i].getAngle());
          }
          currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), states);
        })
        .until(
            () ->
                Arrays.stream(modules)
                    .allMatch(
                        module ->
                            EqualsUtil.epsilonEquals(
                                module.getAngle().getDegrees(),
                                module.getSetpointState().angle.getDegrees(),
                                2.0)))
        .beforeStarting(() -> modulesOrienting = true)
        .finallyDo(() -> modulesOrienting = false)
        .withName("Orient Modules");
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

  public static Rotation2d[] getCircleOrientations() {
    return Arrays.stream(DriveConstants.moduleTranslations)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
  }
}
