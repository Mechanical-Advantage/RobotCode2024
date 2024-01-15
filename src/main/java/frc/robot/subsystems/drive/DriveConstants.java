package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public final class DriveConstants {
  public static final double maxLinearSpeed = Units.feetToMeters(12.16);
  public static final double trackwidthX = Units.inchesToMeters(26.0);
  public static final double trackwidthY = Units.inchesToMeters(26.0);
  public static final double driveBaseRadius = Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
  public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackwidthX / 2.0, trackwidthY / 2.0),
        new Translation2d(trackwidthX / 2.0, -trackwidthY / 2.0),
        new Translation2d(-trackwidthX / 2.0, trackwidthY / 2.0),
        new Translation2d(-trackwidthX / 2.0, -trackwidthY / 2.0)
      };

  // Replace with robots
  public static TrajectoryConstants trajectoryConstants() {
    return switch (Constants.getRobot()) {
      case COMPBOT, KITBOT -> new TrajectoryConstants(6.0, 0.0, 8.0, 0.0);
      case SIMBOT -> new TrajectoryConstants(2.5, 0.0, 7.5, 0.0);
    };
  }

  public record TrajectoryConstants(double drivekp, double drivekd, double turnkp, double turnkd) {}
}
