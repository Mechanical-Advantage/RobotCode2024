// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.ArrayList;
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
import org.littletonrobotics.frc2024.util.Container;

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;
  private final Supplier<List<AutoQuestionResponse>> responses;

  private static final double preloadDelay = 1.0;
  private static final double spikeIntakeDelay = 0.35;
  private static final double spikeFeedThroughDelay = 0.5;
  private static final double stageAimX = FieldConstants.Stage.center.getX() - 0.3;

  private static final Map<Translation2d, HolonomicTrajectory> spiky_firstIntakeReturn =
      loadTrajectorySet("spiky_firstIntakeReturn");
  private static final Map<Translation2d, HolonomicTrajectory> spiky_secondIntakeReturn =
      loadTrajectorySet("spiky_secondIntakeReturn");
  private static final Map<Translation2d, HolonomicTrajectory> spiky_farIntakeReturn =
      loadTrajectorySet("spiky_farIntakeReturn");
  private static final Map<Translation2d, HolonomicTrajectory> CA_secondIntakeReturnToSpikes =
      loadTrajectorySet("spiky_secondIntakeReturnToSpikes");
  private static final Map<Translation2d, HolonomicTrajectory> CA_farIntakeReturnToSpikes =
      loadTrajectorySet("spiky_farIntakeReturnToSpikes");

  private static Map<Translation2d, HolonomicTrajectory> loadTrajectorySet(String name) {
    Map<Translation2d, HolonomicTrajectory> result = new HashMap<>();
    int index = 0;
    while (true) {
      HolonomicTrajectory trajectory;
      try {
        trajectory = new HolonomicTrajectory(name + String.format("%03d", index));
      } catch (Exception e) {
        break;
      }
      result.put(trajectory.getStartPose().getTranslation(), trajectory);
      index++;
    }
    return result;
  }

  /** Command that scores preload. Times out with preloadDelay. */
  private Command spiky_scorePreload() {
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
  private Command spiky_scoreSpikes(AutoQuestionResponse startingLocation, boolean scoresThree) {
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
      default -> {}
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
                                            trajectory.getDuration() + spikeFeedThroughDelay))),
                        () -> scoresThree)))

        // Always aim and run flywheels
        .deadlineWith(superstructure.aimWithCompensation(0.0), flywheels.shootCommand());
  }

  /** Scores two centerline notes with the given trajectories. */
  private Command spiky_scoreCenterlines(HolonomicTrajectory toCenterlineTrajectory) {
    var firstIntakeTrajectory = new HolonomicTrajectory("spiky_firstIntake");
    var secondIntakeTrajectory = new HolonomicTrajectory("spiky_secondIntake");
    var farIntakeTrajectory = new HolonomicTrajectory("spiky_farIntake");

    Timer returnTimer = new Timer();
    Container<HolonomicTrajectory> firstReturnTrajectory = new Container<>();
    Container<HolonomicTrajectory> selectedSecondIntakeTrajectory = new Container<>();
    Container<Map<Translation2d, HolonomicTrajectory>> secondReturnTrajectorySet =
        new Container<>();
    Container<HolonomicTrajectory> secondReturnTrajectory = new Container<>();
    return Commands.runOnce(
            () -> {
              firstReturnTrajectory.value = null;
              selectedSecondIntakeTrajectory.value = null;
              secondReturnTrajectorySet.value = null;
              secondReturnTrajectory.value = null;
            })
        .andThen(
            // Drive sequence
            Commands.sequence(
                    followTrajectory(drive, toCenterlineTrajectory),
                    followTrajectory(drive, firstIntakeTrajectory).until(rollers::isTouchingNote),
                    Commands.runOnce(
                        () -> {
                          // Select return trajectory
                          firstReturnTrajectory.value =
                              spiky_firstIntakeReturn.get(
                                  RobotState.getInstance()
                                      .getTrajectorySetpoint()
                                      .getTranslation()
                                      .nearest(new ArrayList<>(spiky_firstIntakeReturn.keySet())));
                          returnTimer.restart();
                        }),
                    followTrajectory(drive, () -> firstReturnTrajectory.value),
                    Commands.runOnce(
                        () -> {
                          // Select second intake trajectory
                          boolean isFarIntake =
                              RobotState.getInstance().getTrajectorySetpoint().getY()
                                  < FieldConstants.Stage.ampLeg.getY();
                          selectedSecondIntakeTrajectory.value =
                              isFarIntake ? farIntakeTrajectory : secondIntakeTrajectory;
                          secondReturnTrajectorySet.value =
                              isFarIntake ? spiky_farIntakeReturn : spiky_secondIntakeReturn;
                        }),
                    followTrajectory(drive, () -> selectedSecondIntakeTrajectory.value)
                        .until(rollers::isTouchingNote),
                    Commands.runOnce(
                        () -> {
                          // Select return trajectory
                          secondReturnTrajectory.value =
                              secondReturnTrajectorySet.value.get(
                                  RobotState.getInstance()
                                      .getTrajectorySetpoint()
                                      .getTranslation()
                                      .nearest(
                                          new ArrayList<>(
                                              secondReturnTrajectorySet.value.keySet())));
                          returnTimer.restart();
                        }),
                    followTrajectory(drive, () -> secondReturnTrajectory.value))
                .alongWith(
                    // Superstructure and rollers sequence
                    Commands.sequence(
                        // Intake and shoot first centerline
                        waitUntilXCrossed(FieldConstants.wingX, true)
                            .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                            .until(
                                () ->
                                    firstReturnTrajectory.value != null
                                        && returnTimer.hasElapsed(
                                            firstReturnTrajectory.value.getDuration()
                                                - shootTimeoutSecs.get() / 2.0))
                            .andThen(feed(rollers))
                            .deadlineWith(
                                Commands.waitUntil(
                                        () ->
                                            firstReturnTrajectory.value != null
                                                && returnTimer.hasElapsed(
                                                    firstReturnTrajectory.value.getDuration() - 1.5)
                                                && (RobotState.getInstance()
                                                            .getTrajectorySetpoint()
                                                            .getY()
                                                        > FieldConstants.Stage.ampLeg.getY()
                                                    || xCrossed(stageAimX, false)))
                                    .andThen(
                                        Commands.parallel(
                                            aim(drive), superstructure.aimWithCompensation(0.0)))),

                        // Intake and shoot second centerline
                        waitUntilXCrossed(FieldConstants.wingX, true)
                            .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                            .until(
                                () ->
                                    secondReturnTrajectory.value != null
                                        && returnTimer.hasElapsed(
                                            secondReturnTrajectory.value.getDuration()
                                                - shootTimeoutSecs.get() / 2.0))
                            .andThen(feed(rollers))
                            .deadlineWith(
                                Commands.waitUntil(
                                        () ->
                                            secondReturnTrajectory.value != null
                                                && returnTimer.hasElapsed(
                                                    secondReturnTrajectory.value.getDuration()
                                                        - 1.5)
                                                && (RobotState.getInstance()
                                                            .getTrajectorySetpoint()
                                                            .getY()
                                                        > FieldConstants.Stage.ampLeg.getY()
                                                    || xCrossed(stageAimX, false)))
                                    .andThen(
                                        Commands.parallel(
                                            aim(drive), superstructure.aimWithCompensation(0.0))))))
                // Run flywheels
                .deadlineWith(flywheels.shootCommand()));
  }

  /** Returns ending spike index given starting location and whether scoring three spikes or not. */
  private static int spiky_calculateFinalSpikeIndex(
      AutoQuestionResponse startingLocation, boolean scoresThree) {
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

  public Command davisSpikyAuto() {
    List<AutoQuestionResponse> startingLocations =
        List.of(AutoQuestionResponse.SOURCE, AutoQuestionResponse.CENTER, AutoQuestionResponse.AMP);

    // Set up spike choices
    Map<AutoQuestionResponse, Command> spikeChoices = new HashMap<>();
    for (var startingLocation : startingLocations) {
      spikeChoices.put(
          startingLocation,
          Commands.either(
              spiky_scoreSpikes(startingLocation, true),
              spiky_scoreSpikes(startingLocation, false),
              () -> responses.get().get(1).equals(AutoQuestionResponse.THREE)));
    }

    // Set up centerline
    Map<AutoQuestionResponse, Command> centerlineChoices = new HashMap<>();
    for (var startingLocation : startingLocations) {
      HolonomicTrajectory spike3ToFirstIntake =
          new HolonomicTrajectory(
              "spiky_spike"
                  + spiky_calculateFinalSpikeIndex(startingLocation, true)
                  + "ToFirstIntake");
      HolonomicTrajectory spike2ToFirstIntake =
          new HolonomicTrajectory(
              "spiky_spike"
                  + spiky_calculateFinalSpikeIndex(startingLocation, false)
                  + "ToFirstIntake");

      centerlineChoices.put(
          startingLocation,
          Commands.either(
              spiky_scoreCenterlines(spike3ToFirstIntake),
              spiky_scoreCenterlines(spike2ToFirstIntake),
              () -> responses.get().get(1).equals(AutoQuestionResponse.THREE) // Scores three spikes
              ));
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
        spiky_scorePreload(),
        Commands.runOnce(() -> System.out.println("Preload at: " + autoTimer.get())),

        // Score spike notes
        Commands.select(spikeChoices, () -> responses.get().get(0)),
        Commands.runOnce(() -> System.out.println("Spikes finished at: " + autoTimer.get())),

        // Score centerline notes
        Commands.select(centerlineChoices, () -> responses.get().get(0)),
        Commands.runOnce(() -> System.out.println("Centerlines finished at: " + autoTimer.get())));
  }

  public Command davisCAAuto() {
    final HolonomicTrajectory preloadWithSpike2 = new HolonomicTrajectory("CA_startToFirstIntake");
    final HolonomicTrajectory firstIntake = new HolonomicTrajectory("spiky_firstIntake");
    final HolonomicTrajectory secondIntake = new HolonomicTrajectory("spiky_secondIntake");
    final HolonomicTrajectory farIntake = new HolonomicTrajectory("spiky_farIntake");
    final double spike1IntakeTime = 2.5;

    Timer autoTimer = new Timer();
    Timer returnTimer = new Timer();
    Timer remainingSpikesTimer = new Timer();
    Container<HolonomicTrajectory> firstReturnTrajectory = new Container<>();
    Container<HolonomicTrajectory> secondIntakeTrajectory = new Container<>();
    Container<Map<Translation2d, HolonomicTrajectory>> secondReturnTrajectorySet =
        new Container<>();
    Container<HolonomicTrajectory> secondReturnAndSpikesTrajectory = new Container<>();
    Container<Boolean> farTrajectory = new Container<>();
    return Commands.runOnce(
            () -> {
              // Reset timers
              autoTimer.restart();
              remainingSpikesTimer.stop();
              remainingSpikesTimer.reset();
              returnTimer.stop();
              returnTimer.reset();
              // Reset selected
              firstReturnTrajectory.value = null;
              secondIntakeTrajectory.value = null;
              secondReturnTrajectorySet.value = null;
              secondReturnAndSpikesTrajectory.value = null;
              farTrajectory.value = false;
            })
        .andThen(
            // Trajectory sequencing
            Commands.sequence(
                    resetPose(DriveTrajectories.startingAmp),
                    followTrajectory(drive, preloadWithSpike2), // Do preload + spike 2
                    followTrajectory(drive, firstIntake)
                        .until(rollers::isTouchingNote), // Intake from centerline
                    // Return to stageLeftShot
                    Commands.runOnce(
                        () -> {
                          firstReturnTrajectory.value =
                              spiky_firstIntakeReturn.get(
                                  RobotState.getInstance()
                                      .getTrajectorySetpoint()
                                      .getTranslation()
                                      .nearest(new ArrayList<>(spiky_firstIntakeReturn.keySet())));
                          returnTimer.restart();
                        }),
                    followTrajectory(drive, () -> firstReturnTrajectory.value),
                    // Second intake and return
                    Commands.runOnce(
                        () -> {
                          final boolean isFarIntake =
                              RobotState.getInstance().getTrajectorySetpoint().getY()
                                  <= FieldConstants.Stage.ampLeg.getY();
                          farTrajectory.value = true;
                          secondIntakeTrajectory.value = isFarIntake ? farIntake : secondIntake;
                          secondReturnTrajectorySet.value =
                              isFarIntake
                                  ? CA_farIntakeReturnToSpikes
                                  : CA_secondIntakeReturnToSpikes;
                        }),
                    followTrajectory(drive, () -> secondIntakeTrajectory.value)
                        .until(rollers::isTouchingNote),
                    followTrajectory(
                        drive,
                        () -> {
                          secondReturnAndSpikesTrajectory.value =
                              secondReturnTrajectorySet.value.get(
                                  RobotState.getInstance()
                                      .getEstimatedPose()
                                      .getTranslation()
                                      .nearest(
                                          secondReturnTrajectorySet.value.keySet().stream()
                                              .toList()));
                          return secondReturnAndSpikesTrajectory.value;
                        }))
                .alongWith(
                    // Superstructure and rollers seqeuence
                    Commands.sequence(
                            // Score preload and spike 2
                            Commands.waitUntil(() -> autoTimer.hasElapsed(1.0))
                                .andThen(rollers.setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED))
                                .deadlineWith(
                                    Commands.parallel(
                                        aim(drive),
                                        superstructure.setGoalCommand(Superstructure.Goal.AIM)))
                                .until(() -> autoTimer.hasElapsed(2.6)),
                            // Intake and shoot first centerline
                            waitUntilXCrossed(FieldConstants.wingX, true)
                                .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                                .until(
                                    () ->
                                        firstReturnTrajectory.value != null
                                            && returnTimer.hasElapsed(
                                                firstReturnTrajectory.value.getDuration()
                                                    - shootTimeoutSecs.get() / 2.0))
                                .andThen(feed(rollers))
                                .deadlineWith(
                                    Commands.waitUntil(
                                            () ->
                                                firstReturnTrajectory.value != null
                                                    && returnTimer.hasElapsed(
                                                        firstReturnTrajectory.value.getDuration()
                                                            - 1.5))
                                        .andThen(
                                            Commands.parallel(
                                                aim(drive),
                                                superstructure.aimWithCompensation(0.0)))),

                            // Intake and shoot second centerline then score spikes
                            // Rollers sequence
                            Commands.sequence(
                                // Shoot second centerline
                                rollers
                                    .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                    .raceWith(
                                        waitUntilXCrossed(
                                            DriveTrajectories.CA_lastCenterlineShot.getX(), false)),
                                feed(rollers),
                                // Intake and shoot spike 1
                                rollers
                                    .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                    .until(
                                        () ->
                                            remainingSpikesTimer.hasElapsed(
                                                spike1IntakeTime + spikeIntakeDelay)),
                                // Run intake and feeder for spike 0
                                rollers
                                    .setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED)
                                    .until(
                                        () ->
                                            autoTimer.hasElapsed(
                                                preloadWithSpike2.getDuration()
                                                    + firstIntake.getDuration()
                                                    + firstReturnTrajectory.value.getDuration()
                                                    + secondIntakeTrajectory.value.getDuration()
                                                    + secondReturnAndSpikesTrajectory.value
                                                        .getDuration()
                                                    + spikeFeedThroughDelay * 0.5))))
                        .deadlineWith(
                            // Reset timer
                            waitUntilXCrossed(DriveTrajectories.CA_lastCenterlineShot.getX(), false)
                                .andThen(remainingSpikesTimer::restart),
                            // Aim arm
                            waitUntilXCrossed(stageAimX, false)
                                .andThen(superstructure.setGoalCommand(Superstructure.Goal.AIM)),
                            Commands.sequence(
                                // Aim last centerline shot
                                waitUntilXCrossed(
                                        DriveTrajectories.CA_lastCenterlineShot.getX(), false)
                                    .andThen(aim(drive))
                                    .until(
                                        () ->
                                            remainingSpikesTimer.hasElapsed(
                                                shootTimeoutSecs.get() * 1.5)),

                                // Aim spike 1 and 0
                                Commands.waitUntil(
                                        () -> remainingSpikesTimer.hasElapsed(spike1IntakeTime))
                                    .andThen(aim(drive)))))
                // Run flywheels
                .deadlineWith(flywheels.shootCommand()));
  }

  public Command davisSpeedyAuto() {
    var grabCenterline4 = new HolonomicTrajectory("speedy_ampToCenterline4");
    var grabCenterline3 = new HolonomicTrajectory("speedy_centerline4ToCenterline3");
    var grabCenterline2 = new HolonomicTrajectory("speedy_centerline3ToCenterline2");
    var grabEjected = new HolonomicTrajectory("speedy_centerline2ToEjectedNote");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            Commands.sequence(
                    resetPose(DriveTrajectories.startingAmpWall),
                    followTrajectory(drive, grabCenterline4),
                    followTrajectory(drive, grabCenterline3),
                    followTrajectory(drive, grabCenterline2),
                    followTrajectory(drive, grabEjected))
                .alongWith(
                    Commands.sequence(
                            Commands.waitSeconds(1.4),
                            feed(rollers),

                            // Grab and score centerline 4
                            waitUntilXCrossed(FieldConstants.wingX, true)
                                .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                                .until(
                                    () ->
                                        autoTimer.hasElapsed(
                                            grabCenterline4.getDuration()
                                                - shootTimeoutSecs.get() / 2.0))
                                .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                                .deadlineWith(
                                    Commands.waitUntil(
                                            () ->
                                                autoTimer.hasElapsed(
                                                    grabCenterline4.getDuration() - 1.5))
                                        .andThen(
                                            Commands.parallel(
                                                aim(drive),
                                                superstructure.aimWithCompensation(0.0)))),

                            // Grab and score centerline 3
                            waitUntilXCrossed(FieldConstants.wingX, true)
                                .andThen(
                                    rollers
                                        .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                        .until(
                                            () ->
                                                autoTimer.hasElapsed(
                                                    grabCenterline4.getDuration()
                                                        + grabCenterline3.getDuration()
                                                        - shootTimeoutSecs.get() / 2.0))
                                        .andThen(feed(rollers))
                                        .deadlineWith(
                                            Commands.waitUntil(
                                                    () ->
                                                        autoTimer.hasElapsed(
                                                            grabCenterline4.getDuration()
                                                                + grabCenterline3.getDuration()
                                                                - 1.5))
                                                .andThen(
                                                    Commands.parallel(
                                                        aim(drive),
                                                        superstructure.aimWithCompensation(0.0))))),

                            // Grab and score centerline 2
                            waitUntilXCrossed(FieldConstants.wingX, true)
                                .andThen(
                                    rollers
                                        .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                        .until(
                                            () ->
                                                autoTimer.hasElapsed(
                                                    grabCenterline4.getDuration()
                                                        + grabCenterline3.getDuration()
                                                        + grabCenterline2.getDuration()
                                                        - shootTimeoutSecs.get() / 2.0))
                                        .andThen(feed(rollers))
                                        .deadlineWith(
                                            waitUntilXCrossed(stageAimX, false)
                                                .andThen(
                                                    Commands.parallel(
                                                        aim(drive),
                                                        superstructure.aimWithCompensation(0.0))))),
                            // Grab ejected note
                            superstructure
                                .setGoalCommand(Superstructure.Goal.INTAKE)
                                .alongWith(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                                .withTimeout(grabEjected.getDuration() - 0.5),

                            // Score ejected note
                            Commands.waitSeconds(0.65)
                                .andThen(feed(rollers))
                                .deadlineWith(
                                    aim(drive),
                                    superstructure.setGoalCommand(Superstructure.Goal.AIM)))
                        .deadlineWith(
                            flywheels
                                .ejectCommand()
                                .withTimeout(2.0)
                                .andThen(flywheels.shootCommand()))));
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
                                                    grabCenterline0.getDuration() - 0.75))
                                        .andThen(
                                            Commands.parallel(
                                                aim(drive),
                                                superstructure.aimWithCompensation(0.0)))),

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
                                                .andThen(
                                                    Commands.parallel(
                                                        aim(drive),
                                                        superstructure.aimWithCompensation(0.0))))),

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
                                                .andThen(
                                                    Commands.parallel(
                                                        aim(drive),
                                                        superstructure.aimWithCompensation(0.0))))))
                        // Run flywheels
                        .deadlineWith(flywheels.shootCommand())));
  }

  public Command davisUnethicalAuto() {
    Timer autoTimer = new Timer();
    double brakeThreshold = FieldConstants.fieldLength - FieldConstants.startingLineX - 0.5;
    HolonomicTrajectory grabEjected = new HolonomicTrajectory("unethical_grabEjected");
    HolonomicTrajectory driveToSource = new HolonomicTrajectory("unethical_driveToSource");
    Map<AutoQuestionResponse, Command> centerlineChoices = new HashMap<>();

    centerlineChoices.put(
        AutoQuestionResponse.SOURCE_WALL,
        unethical_poopThenScoreCenterlines(
            new HolonomicTrajectory("unethical_grabCenterline0"),
            new HolonomicTrajectory("unethical_centerline0ToCenterline1"),
            0,
            0));
    centerlineChoices.put(
        AutoQuestionResponse.SOURCE_MIDDLE,
        unethical_poopThenScoreCenterlines(
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
            resetPose(DriveTrajectories.startingFarSource),
            Commands.runOnce(autoTimer::restart),
            Commands.select(centerlineChoices, () -> responses.get().get(0)),
            followTrajectory(drive, grabEjected)
                .deadlineWith(
                    intake(superstructure, rollers)
                        .withTimeout(grabEjected.getDuration() - 0.5)
                        .andThen(
                            Commands.parallel(
                                aim(drive),
                                superstructure.setGoalCommand(Superstructure.Goal.AIM)))),
            feed(rollers)
                .deadlineWith(aim(drive), superstructure.setGoalCommand(Superstructure.Goal.AIM)),
            Commands.runOnce(() -> drive.setCoastRequest(Drive.CoastRequest.ALWAYS_COAST)),
            endCoast,
            followTrajectory(drive, driveToSource))
        .deadlineWith(flywheels.ejectCommand().withTimeout(1.0).andThen(flywheels.shootCommand()));
  }

  /** Scores two centerline notes with the given trajectories */
  private Command unethical_poopThenScoreCenterlines(
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
                                    Commands.parallel(
                                        aim(drive),
                                        superstructure.aimWithCompensation(firstShotCompensation))),

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
                                    Commands.parallel(
                                        aim(drive),
                                        superstructure.aimWithCompensation(
                                            secondShotCompensation)))))));
  }
}
