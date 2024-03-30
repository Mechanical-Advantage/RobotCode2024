// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.DriveTrajectories;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;
  private final Supplier<List<AutoQuestionResponse>> responses;

  private int calculateCenterlineIndex(AutoQuestionResponse centerlineNote) {
    return switch (centerlineNote) {
      case AMP_WALL -> 4;
      case AMP_MIDDLE -> 3;
      case MIDDLE -> 2;
      case SOURCE_MIDDLE -> 1;
      case SOURCE -> 0;
      default -> 1;
    };
  }

  private final double preloadDelay = 1.0;

  /** Scores two centerline notes with the given trajectories */
  private Command poopThenScoreCenterlines(
      HolonomicTrajectory startToCenterline1,
      HolonomicTrajectory shotToCenterline2,
      double firstShotCompensation,
      double secondShotCompensation) {
    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive sequence
            Commands.sequence(
                    followTrajectory(drive, startToCenterline1),
                    followTrajectory(drive, shotToCenterline2))
                .alongWith(
                    // Superstructure and rollers sequence
                    Commands.sequence(
                        waitUntilXCrossed(FieldConstants.Stage.center.getX(), true)
                            .deadlineWith(
                                Commands.parallel(
                                    flywheels.ejectCommand(),
                                    superstructure.setGoalCommand(Superstructure.Goal.STOW),
                                    feed(rollers))),
                        Commands.sequence(
                                // Intake centerline 1
                                waitUntilXCrossed(FieldConstants.wingX + 0.85, true)
                                    .andThen(
                                        waitUntilXCrossed(FieldConstants.wingX + 0.8, false)
                                            .deadlineWith(intake(superstructure, rollers))),

                                // Shoot centerline 1
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                startToCenterline1.getDuration()
                                                    - shootTimeoutSecs.get() / 2.0))
                                    .andThen(feed(rollers))
                                    .deadlineWith(
                                        superstructure.aimWithCompensation(firstShotCompensation)),

                                // Intake centerline 2
                                waitUntilXCrossed(FieldConstants.wingX + 0.85, true)
                                    .andThen(
                                        waitUntilXCrossed(FieldConstants.wingX + 0.8, false)
                                            .deadlineWith(intake(superstructure, rollers))),

                                // Shoot centerline 2
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                startToCenterline1.getDuration()
                                                    + shotToCenterline2.getDuration()
                                                    - shootTimeoutSecs.get() / 2.0))
                                    .andThen(feed(rollers))
                                    .deadlineWith(
                                        superstructure.aimWithCompensation(secondShotCompensation)))
                            // Run flywheels
                            .deadlineWith(flywheels.shootCommand()))));
  }

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
    Timer autoTimer = new Timer();
    double sourcePathDelay = 2.5;
    double brakeThreshold = FieldConstants.fieldLength - FieldConstants.startingLineX - 0.5;
    HolonomicTrajectory grabEjected = new HolonomicTrajectory("unethical_grabEjected");
    HolonomicTrajectory driveToSource = new HolonomicTrajectory("unethical_driveToSource");
    Map<AutoQuestionResponse, Command> centerlineChoices = new HashMap<>();

    centerlineChoices.put(
        AutoQuestionResponse.SOURCE_WALL,
        poopThenScoreCenterlines(
            new HolonomicTrajectory("unethical_grabCenterline0"),
            new HolonomicTrajectory("unethical_centerline0ToCenterline1"),
            0,
            0));
    centerlineChoices.put(
        AutoQuestionResponse.SOURCE_MIDDLE,
        poopThenScoreCenterlines(
            new HolonomicTrajectory("unethical_grabCenterline1"),
            new HolonomicTrajectory("unethical_centerline1ToCenterline0"),
            0,
            0));

    ScheduleCommand endCoast =
        new ScheduleCommand(
            Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            AllianceFlipUtil.apply(
                                    RobotState.getInstance().getEstimatedPose().getX())
                                >= brakeThreshold),
                    Commands.runOnce(() -> drive.setCoastRequest(Drive.CoastRequest.ALWAYS_BRAKE)))
                .raceWith(
                    Commands.sequence(
                        Commands.waitUntil(DriverStation::isDisabled), Commands.waitSeconds(3)))
                .ignoringDisable(true));

    return Commands.sequence(
        resetPose(DriveTrajectories.startingDriverStation),
        Commands.runOnce(autoTimer::restart),
        Commands.runOnce(autoTimer::restart),
        Commands.select(centerlineChoices, () -> responses.get().get(0)),
        followTrajectory(drive, grabEjected)
            .deadlineWith(intake(superstructure, rollers))
            .andThen(feed(rollers)),
        Commands.waitUntil(() -> autoTimer.get() >= 15.3 - sourcePathDelay),
        Commands.runOnce(() -> drive.setCoastRequest(Drive.CoastRequest.ALWAYS_COAST)),
        endCoast,
        followTrajectory(drive, driveToSource));
  }
}
