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
  private static final double spikeFeedThroughDelay = 0.35;
  private static final double stageAimX = FieldConstants.Stage.center.getX() - 0.3;

  /** All shot compensation values used when shooting corresponding centerline note. */
  private static final Map<Integer, Double> centerlineShotCompensations =
      Map.of(
          0, 0.0,
          1, 0.0,
          2, 0.0,
          3, 0.0,
          4, 0.0);

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
    final double lastIntakeTime = trajectory.getDuration() - spikeFeedThroughDelay / 2.0;
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

                        // Aim second shot if scoring three spikes
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
                        // If three spike notes
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
                                    () ->
                                        autoTimer.hasElapsed(
                                            lastIntakeTime + spikeFeedThroughDelay))),

                        // If two spike notes
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
                                            lastIntakeTime + spikeFeedThroughDelay))),
                        () -> scoresThree)))

        // Always aim and run flywheels
        .deadlineWith(superstructure.aimWithCompensation(0.0), flywheels.shootCommand())
        .beforeStarting(() -> RobotState.getInstance().setUseAutoLookahead(true))
        .finallyDo(() -> RobotState.getInstance().setUseAutoLookahead(false));
  }

  /** Scores two centerline notes with the given trajectories. */
  private Command scoreCenterlines(
      HolonomicTrajectory spikeToCenterline1,
      HolonomicTrajectory shotToCenterline2,
      double firstShotCompensation,
      double secondShotCompensation) {
    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive sequence
            Commands.sequence(
                    followTrajectory(drive, spikeToCenterline1),
                    followTrajectory(drive, shotToCenterline2))
                .alongWith(
                    // Superstructure and rollers sequence
                    Commands.sequence(
                        // Intake and shoot centerline 1
                        waitUntilXCrossed(FieldConstants.wingX, true)
                            .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        spikeToCenterline1.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
                            .andThen(feed(rollers))
                            .deadlineWith(
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                spikeToCenterline1.getDuration() - 1.5))
                                    .andThen(
                                        superstructure.aimWithCompensation(firstShotCompensation))),

                        // Intake and shoot centerline 1
                        waitUntilXCrossed(FieldConstants.wingX, true)
                            .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        spikeToCenterline1.getDuration()
                                            + shotToCenterline2.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
                            .andThen(feed(rollers))
                            .deadlineWith(
                                Commands.waitUntil(
                                        () ->
                                            autoTimer.hasElapsed(
                                                spikeToCenterline1.getDuration()
                                                    + shotToCenterline2.getDuration()
                                                    - 1.5))
                                    .andThen(
                                        superstructure.aimWithCompensation(
                                            secondShotCompensation)))))
                // Run flywheels
                .deadlineWith(flywheels.shootCommand()));
  }

  /** Returns ending spike index given starting location and whether scoring three spikes or not. */
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

  /** Returns centerline index given centerline note response. */
  private int calculateCenterlineIndex(AutoQuestionResponse centerlineNote) {
    return switch (centerlineNote) {
      case AMP_WALL -> 4;
      case AMP_MIDDLE -> 3;
      case MIDDLE -> 2;
      default -> 1;
    };
  }

  public Command davisSpikyAuto() {
    List<AutoQuestionResponse> centerlineOptions =
        List.of(
            AutoQuestionResponse.AMP_WALL,
            AutoQuestionResponse.AMP_MIDDLE,
            AutoQuestionResponse.MIDDLE);
    List<AutoQuestionResponse> startingLocations =
        List.of(AutoQuestionResponse.SOURCE, AutoQuestionResponse.CENTER, AutoQuestionResponse.AMP);

    // Set up spike choices
    Map<AutoQuestionResponse, Command> spikeChoices = new HashMap<>();
    for (var startingLocation : startingLocations) {
      spikeChoices.put(
          startingLocation,
          Commands.either(
              scoreSpikes(startingLocation, true),
              scoreSpikes(startingLocation, false),
              () -> responses.get().get(1).equals(AutoQuestionResponse.THREE)));
    }

    // Set up centerline choices
    Map<AutoQuestionResponse, Command> centerlinesChoices = new HashMap<>();
    for (var centerline2 : centerlineOptions) {
      int centerline2Index = calculateCenterlineIndex(centerline2);
      Map<AutoQuestionResponse, Command> centerline1Choices = new HashMap<>();

      for (var centerline1 : centerlineOptions) {
        int centerline1Index = calculateCenterlineIndex(centerline1);
        Map<AutoQuestionResponse, Command> startingLocationChoices = new HashMap<>();

        // Add scoreCenterlines command for each centerline 1 choice and centerline 2 choice
        for (var startingLocation : startingLocations) {
          // Get corresponding trajectories
          HolonomicTrajectory spike3ToCenterline1 =
              new HolonomicTrajectory(
                  "spiky_spike"
                      + calculateFinalSpikeIndex(startingLocation, true)
                      + "ToCenterline"
                      + centerline1Index);
          HolonomicTrajectory spike2ToCenterline1 =
              new HolonomicTrajectory(
                  "spiky_spike"
                      + calculateFinalSpikeIndex(startingLocation, false)
                      + "ToCenterline"
                      + centerline1Index);
          HolonomicTrajectory shotToCenterline2 =
              new HolonomicTrajectory("spiky_shotToCenterline" + centerline2Index);

          // Add scoreCenterlines command based on starting location and whether scoring
          // three or two spike notes
          startingLocationChoices.put(
              startingLocation,
              Commands.either(
                  scoreCenterlines(
                      spike3ToCenterline1,
                      shotToCenterline2,
                      centerlineShotCompensations.get(centerline1Index),
                      centerlineShotCompensations.get(centerline2Index)),
                  scoreCenterlines(
                      spike2ToCenterline1,
                      shotToCenterline2,
                      centerlineShotCompensations.get(centerline1Index),
                      centerlineShotCompensations.get(centerline2Index)),
                  () ->
                      responses
                          .get()
                          .get(1)
                          .equals(AutoQuestionResponse.THREE) // Scores three spikes
                  ));
        }
        centerline1Choices.put(
            centerline1, Commands.select(startingLocationChoices, () -> responses.get().get(0)));
      }
      centerlinesChoices.put(
          centerline2, Commands.select(centerline1Choices, () -> responses.get().get(2)));
    }

    int[] validCenterlineIndices = {2, 3, 4};
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
        Commands.select(spikeChoices, () -> responses.get().get(0)),
        Commands.runOnce(() -> System.out.println("Spikes finished at: " + autoTimer.get())),

        // Score centerline notes
        Commands.select(centerlinesChoices, () -> responses.get().get(3)),
        Commands.runOnce(() -> System.out.println("Centerlines finished at: " + autoTimer.get())),

        // Intake missing centerline note
        Commands.select(
                Map.of(
                    2,
                    followTrajectory(drive, new HolonomicTrajectory("spiky_shotToCenterline2")),
                    3,
                    followTrajectory(drive, new HolonomicTrajectory("spiky_shotToCenterline3")),
                    4,
                    followTrajectory(drive, new HolonomicTrajectory("spiky_shotToCenterline4"))),
                () -> {
                  int centerline1 = calculateCenterlineIndex(responses.get().get(2));
                  int centerline2 = calculateCenterlineIndex(responses.get().get(3));
                  int missingCenterline = validCenterlineIndices[0];
                  for (int i = 0; i < 3; i++) {
                    if (missingCenterline != centerline1 && missingCenterline != centerline2) {
                      break;
                    }
                    missingCenterline = validCenterlineIndices[i];
                  }
                  return missingCenterline;
                })
            .deadlineWith(intake(superstructure, rollers)));
  }

  public Command davisSpeedyAuto() {
    return Commands.none();
  }

  public Command davisEthicalAuto() {
    HolonomicTrajectory grabCenterline0 = new HolonomicTrajectory("ethical_grabCenterline0");
    HolonomicTrajectory grabCenterline1 = new HolonomicTrajectory("ethical_grabCenterline1");
    HolonomicTrajectory grabCenterline2 = new HolonomicTrajectory("ethical_grabCenterline2");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive Sequence
            Commands.sequence(
                    resetPose(DriveTrajectories.startingSource),
                    aim(drive).withTimeout(preloadDelay),
                    followTrajectory(drive, grabCenterline0),
                    followTrajectory(drive, grabCenterline1),
                    followTrajectory(drive, grabCenterline2))
                .alongWith(
                    // Superstructure and rollers sequence
                    Commands.sequence(
                            // Score preload
                            Commands.waitUntil(flywheels::atGoal)
                                .andThen(feed(rollers))
                                .deadlineWith(superstructure.aimWithCompensation(0.0))
                                .withTimeout(preloadDelay + shootTimeoutSecs.get()),

                            // Intake and shoot centerline 0
                            waitUntilXCrossed(FieldConstants.wingX, true)
                                .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                                .until(
                                    () ->
                                        autoTimer.hasElapsed(
                                            preloadDelay
                                                + grabCenterline0.getDuration()
                                                - shootTimeoutSecs.get() / 2.0))
                                .andThen(feed(rollers))
                                .deadlineWith(
                                    Commands.waitUntil(
                                            () ->
                                                autoTimer.hasElapsed(
                                                    grabCenterline0.getDuration() - 1.5))
                                        .andThen(superstructure.aimWithCompensation(0.0))),

                            // Intake and shoot centerline 1
                            waitUntilXCrossed(FieldConstants.wingX, true)
                                .andThen(
                                    rollers
                                        .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                        .until(
                                            () ->
                                                autoTimer.hasElapsed(
                                                    preloadDelay
                                                        + grabCenterline0.getDuration()
                                                        + grabCenterline1.getDuration()
                                                        - shootTimeoutSecs.get() / 2.0))
                                        .andThen(feed(rollers))
                                        .deadlineWith(
                                            waitUntilXCrossed(stageAimX, false)
                                                .andThen(superstructure.aimWithCompensation(0.0)))),

                            // Intake and shoot centerline 2
                            waitUntilXCrossed(FieldConstants.wingX, true)
                                .andThen(
                                    rollers
                                        .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                        .until(
                                            () ->
                                                autoTimer.hasElapsed(
                                                    preloadDelay
                                                        + grabCenterline0.getDuration()
                                                        + grabCenterline1.getDuration()
                                                        + grabCenterline2.getDuration()
                                                        - shootTimeoutSecs.get() / 2.0))
                                        .andThen(feed(rollers))
                                        .deadlineWith(
                                            waitUntilXCrossed(stageAimX, false)
                                                .andThen(superstructure.aimWithCompensation(0.0)))))
                        // Run flywheels
                        .deadlineWith(flywheels.shootCommand())));
  }

  public Command davisUnethicalAuto() {
    return Commands.none();
  }
}
