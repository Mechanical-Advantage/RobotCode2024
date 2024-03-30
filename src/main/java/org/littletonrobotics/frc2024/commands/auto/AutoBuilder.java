// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.DriveTrajectories;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;
  private final Supplier<List<AutoQuestionResponse>> responses;

  public Command davisSpikyAuto() {
    return Commands.none();
  }

  public Command davisSpeedyAuto() {
    return Commands.none();
  }

  public Command davisAlternativeSpeedyAuto() {
    return Commands.none();
  }

  public Command davisEthicalAuto() {
    return Commands.none();
  }

  public Command davisUnethicalAuto() {
    return Commands.none();
  }

  public Command davis7NoteAuto() {
    HolonomicTrajectory spikesWithCenterline4 =
        new HolonomicTrajectory("7note_spikesWithCenterline4");
    HolonomicTrajectory centerline3 = new HolonomicTrajectory("7note_centerline3");
    HolonomicTrajectory centerline2 = new HolonomicTrajectory("7note_centerline2");

    final double preloadDelay = 0.8;

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive Sequence
            Commands.sequence(
                    resetPose(DriveTrajectories.startingSourceShifted),
                    followTrajectory(drive, spikesWithCenterline4),
                    followTrajectory(drive, centerline3),
                    followTrajectory(drive, centerline2))
                .alongWith(
                    // Superstructure and rollers sequence
                    Commands.sequence(
                        // Shoot preload and start intaking + feeding
                        Commands.runOnce(() -> RobotState.getInstance().setUseAutoLookahead(true)),
                        Commands.waitSeconds(preloadDelay)
                            .andThen(rollers.setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED))
                            .deadlineWith(superstructure.aimWithCompensation(0.0))
                            .until(() -> autoTimer.hasElapsed(4.5)),
                        Commands.runOnce(() -> RobotState.getInstance().setUseAutoLookahead(false)),

                        // Intake and shoot centerline 4
                        rollers
                            .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        spikesWithCenterline4.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
                            .andThen(feed(rollers))
                            .deadlineWith(
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                spikesWithCenterline4.getDuration() - 1.5))
                                    .andThen(superstructure.aimWithCompensation(0.0))),

                        // Intake and shoot centerline 3
                        rollers
                            .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        spikesWithCenterline4.getDuration()
                                            + centerline3.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
                            .andThen(feed(rollers))
                            .deadlineWith(
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                spikesWithCenterline4.getDuration()
                                                    + centerline3.getDuration()
                                                    - 1.5))
                                    .andThen(superstructure.aimWithCompensation(0.0))),

                        // Intake and shoot centerline 2
                        rollers
                            .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        spikesWithCenterline4.getDuration()
                                            + centerline3.getDuration()
                                            + centerline2.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
                            .andThen(feed(rollers))
                            .deadlineWith(
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                spikesWithCenterline4.getDuration()
                                                    + centerline3.getDuration()
                                                    + centerline2.getDuration()
                                                    - 1.5))
                                    .andThen(superstructure.aimWithCompensation(0.0)))))
                .deadlineWith(flywheels.shootCommand()));
  }
}
