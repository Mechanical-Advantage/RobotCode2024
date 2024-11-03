// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import lombok.Builder;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.util.swerve.ModuleLimits;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  public static final DriveConfig driveConfig =
      switch (Constants.getRobot()) {
        case SIMBOT, COMPBOT ->
            DriveConfig.builder()
                .wheelRadius(Units.inchesToMeters(1.895))
                .trackWidthX(Units.inchesToMeters(20.75))
                .trackWidthY(Units.inchesToMeters(20.75))
                .bumperWidthX(Units.inchesToMeters(37))
                .bumperWidthY(Units.inchesToMeters(33))
                .maxLinearVelocity(Units.feetToMeters(15.0))
                .maxLinearAcceleration(Units.feetToMeters(75.0))
                .maxAngularVelocity(12.0)
                .maxAngularAcceleration(6.0)
                .build();
        case DEVBOT ->
            new DriveConfig(
                Units.inchesToMeters(2.01834634),
                Units.inchesToMeters(20.75),
                Units.inchesToMeters(20.75),
                Units.inchesToMeters(37),
                Units.inchesToMeters(33),
                Units.feetToMeters(12.16),
                Units.feetToMeters(21.32),
                7.93,
                29.89);
      };
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
        new Translation2d(driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0),
        new Translation2d(-driveConfig.trackWidthX() / 2.0, driveConfig.trackWidthY() / 2.0),
        new Translation2d(-driveConfig.trackWidthX() / 2.0, -driveConfig.trackWidthY() / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Odometry Constants
  public static final double odometryFrequency =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case DEVBOT -> 100.0;
        case COMPBOT -> 250.0;
      };

  public static final Matrix<N3, N1> odometryStateStdDevs =
      switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      };

  // Module Constants
  public static final ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConfig[] {
              new ModuleConfig(16, 12, 0, new Rotation2d(-0.81761), true),
              new ModuleConfig(19, 14, 1, new Rotation2d(1.80875), true),
              new ModuleConfig(17, 13, 2, new Rotation2d(-0.48936), true),
              new ModuleConfig(18, 15, 3, new Rotation2d(-1.52578), true)
            };
        case DEVBOT ->
            new ModuleConfig[] {
              new ModuleConfig(15, 11, 0, new Rotation2d(-0.036), true),
              new ModuleConfig(12, 9, 1, new Rotation2d(1.0185), true),
              new ModuleConfig(14, 10, 2, new Rotation2d(1.0705), true),
              new ModuleConfig(13, 8, 3, new Rotation2d(0.7465), true)
            };
        case SIMBOT -> {
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] = new ModuleConfig(0, 0, 0, new Rotation2d(0), false);
          yield configs;
        }
      };

  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConstants(
                5.0,
                0.0,
                1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
                35.0,
                0.0,
                4000.0,
                50.0,
                Mk4iReductions.L3.reduction,
                Mk4iReductions.TURN.reduction);
        case DEVBOT ->
            new ModuleConstants(
                0.1,
                0.13,
                0.0,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L2.reduction,
                Mk4iReductions.TURN.reduction);
        case SIMBOT ->
            new ModuleConstants(
                0.014,
                0.134,
                0.0,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L3.reduction,
                Mk4iReductions.TURN.reduction);
      };

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          driveConfig.maxLinearVelocity(),
          driveConfig.maxLinearAcceleration(),
          Units.degreesToRadians(1080.0));

  public static final ModuleLimits moduleLimitsFlywheelSpinup =
      new ModuleLimits(
          driveConfig.maxLinearVelocity(),
          driveConfig.maxLinearAcceleration() / 2.0,
          Units.degreesToRadians(1080.0));

  // Trajectory Following
  public static final TrajectoryConstants trajectoryConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> TrajectoryConstants.builder().linearkP(8.0).thetakP(4.0).build();
        case DEVBOT ->
            new TrajectoryConstants(
                6.0,
                0.0,
                8.0,
                0.0,
                Units.inchesToMeters(4.0),
                Units.degreesToRadians(5.0),
                Units.inchesToMeters(5.0),
                Units.degreesToRadians(7.0),
                driveConfig.maxLinearVelocity() / 2.0,
                driveConfig.maxAngularVelocity() / 2.0);
        case SIMBOT ->
            new TrajectoryConstants(
                4.0,
                0.0,
                4.0,
                0.0,
                Units.inchesToMeters(4.0),
                Units.degreesToRadians(5.0),
                Units.inchesToMeters(5.0),
                Units.degreesToRadians(7.0),
                driveConfig.maxLinearVelocity() / 2.0,
                driveConfig.maxAngularVelocity() / 2.0);
      };

  // Swerve Heading Control
  public static final HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        default -> new HeadingControllerConstants(5.0, 0.0, 8.0, 20.0);
      };

  @Builder
  public record DriveConfig(
      double wheelRadius,
      double trackWidthX,
      double trackWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public record ModuleConstants(
      double ffkS,
      double ffkV,
      double ffkT,
      double drivekP,
      double drivekD,
      double turnkP,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  @Builder
  public record TrajectoryConstants(
      double linearkP,
      double linearkD,
      double thetakP,
      double thetakD,
      double linearTolerance,
      double thetaTolerance,
      double goalLinearTolerance,
      double goalThetaTolerance,
      double linearVelocityTolerance,
      double angularVelocityTolerance) {}

  public record AutoAlignConstants(
      double linearkP,
      double linearkD,
      double thetakP,
      double thetakD,
      double linearTolerance,
      double thetaTolerance,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {}

  public record HeadingControllerConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration) {}

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
