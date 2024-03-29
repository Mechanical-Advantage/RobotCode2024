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

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;
  private final Supplier<List<AutoQuestionResponse>> responses;

  private static final double preloadDelay = 1.0;
  private static final double spikeIntakeDelay = 0.35;

  /** Command that scores preload. Times out with preloadDelay. */
  private Command scorePreload() {
    return
    // Aim robot at speaker
    aim(drive)
        .withTimeout(preloadDelay)
        .alongWith(
            // Aim superstructure and shoot preload
            Commands.waitUntil(flywheels::atGoal)
                .andThen(feed(rollers))
                .deadlineWith(superstructure.aimWithCompensation(0.0))
                .withTimeout(preloadDelay))
        // Run flywheels
        .deadlineWith(flywheels.shootCommand());
  }

  /** Scores 2 or 3 spike notes. */
  private Command scoreSpikes(AutoQuestionResponse startingLocation, boolean scoresThree) {
    HolonomicTrajectory trajectory =
        new HolonomicTrajectory(
            "spiky_" + startingLocation.toString().toLowerCase() + "Start" + (scoresThree ? 3 : 2));
    final double lastIntakeTime = trajectory.getDuration() - spikeIntakeDelay / 2.0;
    double firstIntakeTime = 0;
    double secondIntakeTime = lastIntakeTime;
    switch (startingLocation) {
      case SOURCE, AMP -> {
        firstIntakeTime = 1.2;
        secondIntakeTime = 3.0;
      }
      case CENTER -> {
        firstIntakeTime = 1.3;
        secondIntakeTime = 3.0;
      }
    }

    Timer autoTimer = new Timer();
    final double finalFirstIntakeTime = firstIntakeTime;
    final double finalSecondIntakeTime = secondIntakeTime;
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            followTrajectory(drive, trajectory)
                .alongWith(
                    // Sequence aiming portions
                    Commands.sequence(
                        // Aim first shot
                        Commands.waitUntil(() -> autoTimer.hasElapsed(finalFirstIntakeTime))
                            .andThen(
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                finalFirstIntakeTime
                                                    + spikeIntakeDelay
                                                    + shootTimeoutSecs.get()))
                                    .deadlineWith(aim(drive))),

                        // Aim second shot if trajectory doesn't end there
                        Commands.waitUntil(() -> autoTimer.hasElapsed(finalSecondIntakeTime))
                            .andThen(
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                finalSecondIntakeTime
                                                    + spikeIntakeDelay
                                                    + shootTimeoutSecs.get()))
                                    .deadlineWith(aim(drive)))
                            .onlyIf(() -> scoresThree)),

                    // Sequence feeding and intaking
                    Commands.either(
                        Commands.sequence(
                            // Intake and shoot first spike
                            rollers
                                .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                .until(
                                    () ->
                                        autoTimer.hasElapsed(
                                            finalFirstIntakeTime + spikeIntakeDelay)),
                            feed(rollers),

                            // Intake and shoot second spike
                            rollers
                                .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                .until(
                                    () ->
                                        autoTimer.hasElapsed(
                                            finalSecondIntakeTime + spikeIntakeDelay)),
                            feed(rollers),

                            // Intake and shoot third spike
                            rollers
                                .setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED)
                                .until(
                                    () -> autoTimer.hasElapsed(lastIntakeTime + spikeIntakeDelay))),
                        Commands.sequence(
                            // Intake and shoot first spike
                            rollers
                                .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                .until(
                                    () ->
                                        autoTimer.hasElapsed(
                                            finalFirstIntakeTime + spikeIntakeDelay)),
                            feed(rollers),

                            // Intake and shoot second spike
                            rollers
                                .setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED)
                                .until(
                                    () ->
                                        autoTimer.hasElapsed(
                                            lastIntakeTime
                                                + spikeIntakeDelay
                                                + shootTimeoutSecs.get()))),
                        () -> scoresThree)))

        // Always aim and run flywheels
        .deadlineWith(superstructure.aimWithCompensation(0.0), flywheels.shootCommand())
        .beforeStarting(() -> RobotState.getInstance().setUseAutoLookahead(true))
        .finallyDo(() -> RobotState.getInstance().setUseAutoLookahead(false));
  }

  /** Scores a note from the centerline given a trajectory. */
  private Command scoreCenterline(HolonomicTrajectory trajectory) {
    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            Commands.parallel(
                followTrajectory(drive, trajectory),
                // Sequence superstructure and rollers
                Commands.sequence(
                    // Intake
                    waitUntilXCrossed(FieldConstants.wingX + 0.85, true)
                        .andThen(
                            waitUntilXCrossed(FieldConstants.wingX + 0.8, false)
                                .deadlineWith(intake(superstructure, rollers))),

                    // Shoot
                    Commands.waitUntil(
                            () ->
                                autoTimer.hasElapsed(
                                    trajectory.getDuration() - shootTimeoutSecs.get() / 2.0))
                        .andThen(feed(rollers))
                        .deadlineWith(
                            Commands.waitUntil(
                                    () -> autoTimer.hasElapsed(trajectory.getDuration() - 2.0))
                                .andThen(superstructure.aimWithCompensation(0.0))))))
        .deadlineWith(flywheels.shootCommand());
  }

  private int calculateFinalSpikeIndex(AutoQuestionResponse startingLocation, boolean scoresThree) {
    if (scoresThree) {
      if (startingLocation.equals(AutoQuestionResponse.AMP)) {
        return 0;
      } else {
        return 2;
      }
    } else {
      if (startingLocation.equals(AutoQuestionResponse.CENTER)) {
        return 2;
      } else {
        return 1;
      }
    }
  }

  private int calculateCenterlineIndex(AutoQuestionResponse centerlineNote) {
    return switch (centerlineNote) {
      case AMP_WALL -> 4;
      case AMP_MIDDLE -> 3;
      case MIDDLE -> 2;
      default -> 1;
    };
  }

  public Command davisSpikyAuto() {
    Map<AutoQuestionResponse, Command> spikeChooser = new HashMap<>();
    List<AutoQuestionResponse> startingLocations =
        List.of(AutoQuestionResponse.SOURCE, AutoQuestionResponse.CENTER, AutoQuestionResponse.AMP);
    for (var startingLocation : startingLocations) {
      spikeChooser.put(
          startingLocation,
          Commands.either(
              scoreSpikes(startingLocation, true),
              scoreSpikes(startingLocation, false),
              () -> responses.get().get(1).equals(AutoQuestionResponse.THREE)));
    }

    Map<AutoQuestionResponse, Command> firstCenterlineChooser = new HashMap<>();
    Map<AutoQuestionResponse, Command> secondCenterlineChooser = new HashMap<>();
    List<AutoQuestionResponse> centerlineNotes =
        List.of(
            AutoQuestionResponse.AMP_WALL,
            AutoQuestionResponse.AMP_MIDDLE,
            AutoQuestionResponse.MIDDLE);
    for (var centerlineNote : centerlineNotes) {
      Map<AutoQuestionResponse, Command> spikes = new HashMap<>();
      for (var startingLocation : startingLocations) {
        spikes.put(
            startingLocation,
            Commands.either(
                scoreCenterline(
                    new HolonomicTrajectory(
                        "spiky_spike"
                            + calculateFinalSpikeIndex(startingLocation, true)
                            + "ToCenterline"
                            + calculateCenterlineIndex(centerlineNote))),
                scoreCenterline(
                    new HolonomicTrajectory(
                        "spiky_spike"
                            + calculateFinalSpikeIndex(startingLocation, false)
                            + "ToCenterline"
                            + calculateCenterlineIndex(centerlineNote))),
                () -> responses.get().get(1).equals(AutoQuestionResponse.THREE)));
      }
      firstCenterlineChooser.put(
          centerlineNote, Commands.select(spikes, () -> responses.get().get(0)));
      secondCenterlineChooser.put(
          centerlineNote,
          scoreCenterline(
              new HolonomicTrajectory(
                  "spiky_shotToCenterline" + calculateCenterlineIndex(centerlineNote))));
    }

    Timer autoTimer = new Timer();
    return Commands.sequence(
        Commands.runOnce(autoTimer::restart),
        // Reset pose and shoot preload
        Commands.select(
            Map.of(
                AutoQuestionResponse.SOURCE,
                resetPose(DriveTrajectories.startingSource),
                AutoQuestionResponse.CENTER,
                resetPose(DriveTrajectories.startingCenter),
                AutoQuestionResponse.AMP,
                resetPose(DriveTrajectories.startingAmp)),
            () -> responses.get().get(0)),
        scorePreload(),
        Commands.runOnce(() -> System.out.println("Preload at: " + autoTimer.get())),
        // Score spike notes
        Commands.select(spikeChooser, () -> responses.get().get(0)),
        Commands.runOnce(() -> System.out.println("Spikes finished at: " + autoTimer.get())),
        // Score first centerline note
        Commands.select(firstCenterlineChooser, () -> responses.get().get(2)),
        Commands.runOnce(() -> System.out.println("Centerline 1 shot at: " + autoTimer.get())),
        // Score second centerline note
        Commands.select(secondCenterlineChooser, () -> responses.get().get(3)),
        Commands.runOnce(() -> System.out.println("Centerline 2 shot at: " + autoTimer.get())),
        // Drive to centerline
        followTrajectory(drive, new HolonomicTrajectory("spiky_shotToCenter"))
            .deadlineWith(intake(superstructure, rollers)));
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
}
