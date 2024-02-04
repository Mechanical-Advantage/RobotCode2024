package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SuperstructureConstants {

  public static class ShooterConstants {
    // encoder / flywheelReduction = flywheel
    public static double flywheelReduction = (1.0 / 2.0);
    public static double shooterToleranceRPM = 50.0;

    public static FlywheelConstants leftFlywheelConstants =
        switch (Constants.getRobot()) {
          default -> new FlywheelConstants(2, false, 0.0, 0.0, 0.0, 0.33329, 0.00083, 0.0);
        };

    public static FlywheelConstants rightFlywheelConstants =
        switch (Constants.getRobot()) {
          default -> new FlywheelConstants(1, false, 0.0, 0.0, 0.0, 0.33329, 0.00083, 0.0);
        };

    public static FeederConstants feederConstants =
        switch (Constants.getRobot()) {
          default -> new FeederConstants(3, false);
        };

    public record FlywheelConstants(
        int id,
        boolean inverted,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA) {}

    public record FeederConstants(int id, boolean inverted) {}
  }

  public static class IntakeConstants {
    public static double reduction = (1.0 / 1.0);
    public static int id =
        switch (Constants.getRobot()) {
          default -> 45;
        };
    public static boolean inverted =
        switch (Constants.getRobot()) {
          default -> true;
        };
  }

  public static class ArmConstants {
    // reduction is 12:62 18:60 12:65
    public static double reduction = (62.0 / 12.0) * (60.0 / 18.0) * (65.0 / 12.0);
    public static Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);
    public static Translation2d armOrigin2d =
        new Translation2d(-Units.inchesToMeters(9.37), Units.inchesToMeters(11.75));

    public static Rotation2d minAngle = Rotation2d.fromDegrees(0.0);
    public static Rotation2d maxAngle = Rotation2d.fromDegrees(110.0);

    public static int leaderID =
        switch (Constants.getRobot()) {
          default -> 25;
        };

    public static int followerID =
        switch (Constants.getRobot()) {
          default -> 26;
        };

    public static boolean leaderInverted =
        switch (Constants.getRobot()) {
          case RAINBOWT -> false;
          case COMPBOT -> false;
          case SIMBOT -> false;
        };

    public static boolean followerInverted =
        switch (Constants.getRobot()) {
          case RAINBOWT -> false;
          case COMPBOT -> false;
          case SIMBOT -> false;
        };

    public static double armLength =
        switch (Constants.getRobot()) {
          case RAINBOWT -> Units.inchesToMeters(24.8);
          default -> Units.inchesToMeters(25.866);
        };

    public static ControllerConstants controllerConstants =
        switch (Constants.getRobot()) {
          case SIMBOT -> new ControllerConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          case RAINBOWT -> new ControllerConstants(0.0, 0.0, 0.0, 0.0, 1.84, 0.0001, 0.13);
          case COMPBOT -> new ControllerConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        };

    public static ProfileConstraints profileConstraints =
        switch (Constants.getRobot()) {
          default -> new ProfileConstraints(2.0 * Math.PI, 2.0 * Math.PI);
        };

    public record ProfileConstraints(
        double cruiseVelocityRadPerSec, double accelerationRadPerSec2) {}

    public record ControllerConstants(
        double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
  }
}
