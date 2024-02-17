// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
   * Resets pose to beginning of trajectory using {@link
   * org.littletonrobotics.frc2024.util.AllianceFlipUtil}
   *
   * @param trajectory pose to reset to
   */
  public static Command resetPose(HolonomicTrajectory trajectory) {
    return Commands.runOnce(
        () ->
            RobotState.getInstance().resetPose(AllianceFlipUtil.apply(trajectory.getStartPose())));
  }

  /** Creates a command that follows a trajectory, ending when the trajectory is finished */
  public static Command followTrajectory(Drive drive, HolonomicTrajectory trajectory) {
    return startEnd(() -> drive.setTrajectoryGoal(trajectory), () -> drive.clearTrajectoryGoal())
        .until(drive::isTrajectoryGoalCompleted);
  }

  /**
   * Waits until robot crossed x position. <br>
   *
   * @param xPosition position coordinate on blue side of field.
   */
  public static Command waitUntilXCrossed(double xPosition) {
    double flippedX = AllianceFlipUtil.apply(xPosition);
    return Commands.waitUntil(
        () -> {
          Pose2d robot = RobotState.getInstance().getEstimatedPose();
          if (AllianceFlipUtil.shouldFlip()) {
            return robot.getX() < flippedX;
          } else {
            return robot.getX() > flippedX;
          }
        });
  }

  /** Runs intake until the gamepiece is collected */
  public static Command intake(Superstructure superstructure, Rollers rollers) {
    return superstructure
        .intake()
        .raceWith(
            Commands.waitUntil(superstructure::atGoal)
                .andThen(rollers.floorIntake())
                .until(rollers::hasGamepiece));
  }

  /** Shoots note. */
  public static Command shoot(Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    return Commands.race(
        superstructure.aim(),
        flywheels.shootCommand(),
        Commands.waitUntil(() -> superstructure.atGoal() && flywheels.atGoal())
            .andThen(rollers.feedShooter().withTimeout(shootTimeoutSecs.get())));
  }
}
