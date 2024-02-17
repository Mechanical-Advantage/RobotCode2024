// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private static final LoggedTunableNumber shootTimeoutSecs =
      new LoggedTunableNumber("Auto/ShotTimeoutSecs", 0.5);

  /**
   * Resets pose to beginning of trajectory accounting for alliance color {@link
   * org.littletonrobotics.frc2024.util.AllianceFlipUtil}.
   *
   * @param trajectory Trajectory to reset to.
   */
  public static Command resetPose(HolonomicTrajectory trajectory) {
    return Commands.runOnce(
        () ->
            RobotState.getInstance().resetPose(AllianceFlipUtil.apply(trajectory.getStartPose())));
  }

  /** Creates a command that follows a trajectory, command ends when the trajectory is finished */
  public static Command followTrajectory(Drive drive, HolonomicTrajectory trajectory) {
    return startEnd(() -> drive.setTrajectoryGoal(trajectory), drive::clearTrajectoryGoal)
        .until(drive::isTrajectoryGoalCompleted);
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
            superstructure.intake(), rollers.floorIntake().beforeStarting(superstructure::atGoal))
        .until(rollers::hasGamepiece);
  }

  /** Shoots note, ending after rollers have spun */
  public static Command shoot(
      Drive drive, Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    return parallel(
            // Aim and spin up flywheels
            startEnd(drive::setAutoAimGoal, drive::clearAutoAimGoal),
            superstructure.aim(),
            flywheels.shootCommand())
        // End command when ready to shoot and rollers have spun
        .raceWith(
            Commands.waitUntil(
                    () ->
                        drive.isAutoAimGoalCompleted()
                            && superstructure.atGoal()
                            && flywheels.atGoal())
                .andThen(rollers.feedShooter().withTimeout(shootTimeoutSecs.get())));
  }
}
