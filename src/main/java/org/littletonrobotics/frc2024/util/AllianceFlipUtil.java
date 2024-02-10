package org.littletonrobotics.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.commands.auto.AutoCommands;
import org.littletonrobotics.frc2024.util.trajectory.HolonomicTrajectory;

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

    public static Translation3d apply(Translation3d translation3d) {
        if (shouldFlip()) {
            return new Translation3d(
                    apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
        } else {
            return translation3d;
        }
    }

    /**
     * Flips a trajectory state to the correct side of the field based on the current alliance color.
     */
    public static HolonomicTrajectory.State apply(HolonomicTrajectory.State state) {
        if (shouldFlip()) {
            return new HolonomicTrajectory.State(
                    state.timeSeconds(),
                    apply(state.pose()),
                    -state.velocityX(),
                    state.velocityY(),
                    -state.angularVelocity());
        } else {
            return state;
        }
    }

    /**
     * Flip rectangular region to the correct side of the field based on the current alliance color
     */
    public static AutoCommands.RectangularRegion apply(AutoCommands.RectangularRegion region) {
        if (shouldFlip()) {
            Translation2d topRight = apply(region.topLeft);
            Translation2d bottomLeft = apply(region.bottomRight);
            return new AutoCommands.RectangularRegion(
                    new Translation2d(bottomLeft.getX(), topRight.getY()),
                    new Translation2d(topRight.getX(), bottomLeft.getY()));
        } else {
            return region;
        }
    }

    /** Flip circular region to the correct side of the field based on the current alliance color */
    public static AutoCommands.CircularRegion apply(AutoCommands.CircularRegion region) {
        if (shouldFlip()) {
            return new AutoCommands.CircularRegion(apply(region.center()), region.radius());
        } else {
            return region;
        }
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    }
}
