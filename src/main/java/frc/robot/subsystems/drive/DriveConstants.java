package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  public static DrivetrainConfig drivetrainConfig =
      switch (Constants.getRobot()) {
        default ->
            new DrivetrainConfig(
                Units.inchesToMeters(2.0),
                Units.inchesToMeters(26.0),
                Units.inchesToMeters(26.0),
                Units.feetToMeters(12.16),
                Units.feetToMeters(21.32),
                7.93,
                29.89);
      };
  public static final double wheelRadius = Units.inchesToMeters(2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);
  public static final double odometryFrequency =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case RAINBOWT -> 100.0;
        case COMPBOT -> 250.0;
      };
  public static final Matrix<N3, N1> odometryStateStdDevs =
      switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.1, 0.1, 0.1));
      };

  public static ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT, RAINBOWT ->
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

  public static ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT, RAINBOWT ->
            new ModuleConstants(
                0.1,
                0.13,
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
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L2.reduction,
                Mk4iReductions.TURN.reduction);
      };

  public static TrajectoryConstants trajectoryConstants =
      switch (Constants.getRobot()) {
        case COMPBOT, RAINBOWT ->
            new TrajectoryConstants(
                6.0,
                0.0,
                10.0,
                0.0,
                Units.inchesToMeters(4.0),
                Units.degreesToRadians(5.0),
                Units.inchesToMeters(12.0),
                Units.degreesToRadians(30.0),
                drivetrainConfig.maxLinearVelocity(),
                drivetrainConfig.maxAngularVelocity());
        case SIMBOT ->
            new TrajectoryConstants(
                5.0,
                0.0,
                4.0,
                0.0001,
                Units.inchesToMeters(4.0),
                Units.degreesToRadians(5.0),
                Units.inchesToMeters(12.0),
                Units.degreesToRadians(30.0),
                drivetrainConfig.maxLinearVelocity(),
                drivetrainConfig.maxAngularVelocity());
      };

  public static HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        case COMPBOT, RAINBOWT -> new HeadingControllerConstants(8.0, 0.0);
        case SIMBOT -> new HeadingControllerConstants(4.0, 0.0);
      };

  public record DrivetrainConfig(
      double wheelRadius,
      double trackwidthX,
      double trackwidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public record ModuleConstants(
      double ffKs,
      double ffKv,
      double driveKp,
      double drivekD,
      double turnKp,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  public record TrajectoryConstants(
      double linearKp,
      double linearKd,
      double thetaKp,
      double thetaKd,
      double linearTolerance,
      double thetaTolerance,
      double goalLinearTolerance,
      double goalThetaTolerance,
      double linearVelocityTolerance,
      double angularVelocityTolerance) {}

  public record HeadingControllerConstants(double Kp, double Kd) {}

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
