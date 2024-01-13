package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.util.trajectory.HolonomicDriveController;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(apply(translation.getX()), translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   */
  public static HolonomicDriveController.HolonomicDriveState apply(
      HolonomicDriveController.HolonomicDriveState state) {
    if (shouldFlip()) {
      return new HolonomicDriveController.HolonomicDriveState(
          apply(state.pose()), -state.velocityX(), state.velocityY(), -state.angularVelocity());
    } else {
      return state;
    }
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
