// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

public class AutoCommands {
  public static final LoggedTunableNumber shootTimeoutSecs =
      new LoggedTunableNumber("Auto/ShotTimeoutSecs", 0.3);

  /**
   * Resets pose accounting for alliance color.
   *
   * @param pose Pose to reset to.
   */
  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(() -> RobotState.getInstance().resetPose(AllianceFlipUtil.apply(pose)));
  }

  /**
   * Resets pose to beginning of trajectory accounting for alliance color {@link
   * org.littletonrobotics.frc2024.util.AllianceFlipUtil}.
   *
   * @param trajectory Trajectory to reset to.
   */
  public static Command resetPose(HolonomicTrajectory trajectory) {
    return resetPose(trajectory.getStartPose());
  }

  /** Creates a command that follows a trajectory, command ends when the trajectory is finished */
  public static Command followTrajectory(Drive drive, HolonomicTrajectory trajectory) {
    return startEnd(() -> drive.setTrajectory(trajectory), drive::clearTrajectory)
        .until(drive::isTrajectoryCompleted);
  }

  /**
   * Returns whether robot has crossed x boundary, accounting for alliance flip
   *
   * @param xPosition X position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed x coordinate towards center line or away
   *     from center line
   */
  public static boolean xCrossed(double xPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getX() < FieldConstants.fieldLength - xPosition;
      } else {
        return robotPose.getX() > FieldConstants.fieldLength - xPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getX() > xPosition;
      } else {
        return robotPose.getX() < xPosition;
      }
    }
  }

  /** Command that waits for x boundary to be crossed. See {@link #xCrossed(double, boolean)} */
  public static Command waitUntilXCrossed(double xPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenterline));
  }

  /** Runs intake until the gamepiece is collected, does not end in sim */
  public static Command intake(Superstructure superstructure, Rollers rollers) {
    return parallel(
        superstructure.intake(),
        Commands.waitUntil(superstructure::atGoal)
            .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE)));
  }

  /** Shoots note, ending after rollers have spun */
  public static Command shoot(
      Drive drive, Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    Supplier<Rotation2d> aimHeadingSupplier =
        () -> RobotState.getInstance().getAimingParameters().driveHeading();
    return parallel(
            // Aim and spin up flywheels
            startEnd(() -> drive.setHeadingGoal(aimHeadingSupplier), drive::clearHeadingGoal),
            superstructure.aim())
        // End command when ready to shoot and rollers have spun
        .raceWith(
            Commands.waitUntil(
                    () -> drive.atHeadingGoal() && superstructure.atGoal() && flywheels.atGoal())
                .andThen(
                    rollers
                        .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                        .withTimeout(shootTimeoutSecs.get())));
  }

  public static Command shootNoDrive(
      Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    return parallel(superstructure.aim())
        .raceWith(
            Commands.waitUntil(() -> superstructure.atGoal() && flywheels.atGoal())
                .andThen(
                    rollers
                        .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                        .withTimeout(shootTimeoutSecs.get())));
  }

  /**
   * Runs intake and feeder while aiming at speaker essentially shooting immediately after acquiring
   * game piece. Command does not end.
   */
  public static Command intakeIntoEject(Flywheels flywheels, Rollers rollers) {
    return parallel(
        // Aim and spin up flywheels
        flywheels.ejectCommand(), rollers.setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED));
  }

  // reset Path and call followTrajectory
  public static Command resetAndFollow(Drive drive, HolonomicTrajectory trajectory) {
    return sequence(resetPose(trajectory), followTrajectory(drive, trajectory));
  }
}
