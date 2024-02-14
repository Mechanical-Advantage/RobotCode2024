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

import static frc.robot.Constants.Mode.SIM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);
  public static final double TRACK_WIDTH = Units.inchesToMeters(26.0);
  public static final double MAX_SPEED_M_PER_S = Units.feetToMeters(10);
  private static final double lKS = Constants.currentMode == SIM ? 0.0 : 0.98355;
  private static final double lKV = Constants.currentMode == SIM ? 0.227 : 30.42797;

  private static final double rKS = Constants.currentMode == SIM ? 0.0 : 0.88676;
  private static final double rKV = Constants.currentMode == SIM ? 0.227 : 32.08170;
  public PIDController pid = new PIDController(7, 0, 0);
  private final DriveIO io;
  private final GyroIO gyroIO;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH);
  private final SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(lKS, lKV);
  private final SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(rKS, rKV);

  /** Creates a new Drive. */
  public Drive(DriveIO io, GyroIO gyroIO) {
    this.io = io;
    this.gyroIO = gyroIO;

    AutoBuilder.configureRamsete(
        this::getPose,
        this::setPose,
        () ->
            kinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(
                    getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
        (speeds) -> {
          var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
          driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        },
        new ReplanningConfig(),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs, gyroInputs);
    gyroIO.updateInputs(gyroInputs);

    Logger.processInputs("Drive", inputs);
    Logger.processInputs("Gyro", gyroInputs);

    // Update odometry
    odometry.update(
        Rotation2d.fromRadians(gyroInputs.yawPositionRad),
        getLeftPositionMeters(),
        getRightPositionMeters());
  }

  /** Run open loop at the specified voltage. */
  public void driveVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /** Run closed loop at the specified voltage. */
  public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);

    io.setVoltage(
        pid.calculate(inputs.leftVelocityRadPerSec * WHEEL_RADIUS, leftMetersPerSec)
            + feedforwardLeft.calculate(leftMetersPerSec),
        pid.calculate(inputs.rightVelocityRadPerSec * WHEEL_RADIUS, rightMetersPerSec)
            + feedforwardRight.calculate(rightMetersPerSec));
  }

  /** Run open loop based on stick positions. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    driveVelocity(speeds.left * MAX_SPEED_M_PER_S, speeds.right * MAX_SPEED_M_PER_S);
  }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0.0, 0.0);
  }

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromRadians(gyroInputs.yawPositionRad),
        getLeftPositionMeters(),
        getRightPositionMeters(),
        pose);
    gyroIO.setGyro(gyroInputs, pose.getRotation().getDegrees());
  }

  /** Returns the position of the left wheels in meters. */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * WHEEL_RADIUS;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * WHEEL_RADIUS;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * WHEEL_RADIUS;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * WHEEL_RADIUS;
  }

  @AutoLogOutput
  public double getGyroYawDegrees() {
    return Math.toDegrees(gyroInputs.yawPositionRad);
  }

  public GyroIO getGyroIO() {
    return gyroIO;
  }

  /** Returns the average velocity in meters/second. */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0 * WHEEL_RADIUS;
  }
}
