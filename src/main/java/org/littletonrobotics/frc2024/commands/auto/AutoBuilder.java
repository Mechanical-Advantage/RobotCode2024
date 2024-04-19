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
import java.util.function.BooleanSupplier;
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

  private static final Map<Translation2d, HolonomicTrajectory> thinking_firstIntakeReturn =
      loadTrajectorySet("thinking_firstIntakeReturn");
  private static final Map<Translation2d, HolonomicTrajectory> thinking_secondIntakeReturn =
      loadTrajectorySet("thinking_secondIntakeReturn");
  private static final Map<Translation2d, HolonomicTrajectory> thinking_farIntakeReturn =
      loadTrajectorySet("thinking_farIntakeReturn");
  private static final Map<Translation2d, HolonomicTrajectory> thinking_secondIntakeCAReturn =
      loadTrajectorySet("thinking_secondIntakeCAReturn");
  private static final Map<Translation2d, HolonomicTrajectory> thinking_farIntakeCAReturn =
      loadTrajectorySet("thinking_farIntakeCAReturn");

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

  /** Scores two centerline notes while thinking-on-your-feet. * */
  private Command thinkingOnYourFeet(
      boolean isInfinite,
      boolean isCAReturn,
      BooleanSupplier cancelFirstIntake,
      BooleanSupplier cancelSecondIntake) {
    var firstIntakeTrajectory = new HolonomicTrajectory("thinking_firstIntake");
    var secondIntakeTrajectory = new HolonomicTrajectory("thinking_secondIntake");
    var farIntakeTrajectory = new HolonomicTrajectory("thinking_farIntake");

    Timer returnTimer = new Timer();
    Container<HolonomicTrajectory> firstReturnTrajectory = new Container<>();
    Container<HolonomicTrajectory> selectedSecondIntakeTrajectory = new Container<>();
    Container<Map<Translation2d, HolonomicTrajectory>> secondReturnTrajectorySet =
        new Container<>();
    Container<HolonomicTrajectory> secondReturnTrajectory = new Container<>();

    // Subroutines for second intake (repeated infinitely if desired)
    Command secondIntakeDrive =
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  // Select second intake trajectory
                  boolean isFarIntake =
                      RobotState.getInstance().getTrajectorySetpoint().getY()
                          < FieldConstants.Stage.ampLeg.getY();
                  selectedSecondIntakeTrajectory.value =
                      isFarIntake ? farIntakeTrajectory : secondIntakeTrajectory;
                  if (isCAReturn) {
                    secondReturnTrajectorySet.value =
                        isFarIntake ? thinking_farIntakeCAReturn : thinking_secondIntakeCAReturn;
                  } else {
                    secondReturnTrajectorySet.value =
                        isFarIntake ? thinking_farIntakeReturn : thinking_secondIntakeReturn;
                  }
                  secondReturnTrajectory.value = null;
                }),
            followTrajectory(drive, () -> selectedSecondIntakeTrajectory.value)
                .until(() -> rollers.isTouchingNote() || cancelSecondIntake.getAsBoolean()),
            Commands.runOnce(
                () -> {
                  // Select return trajectory
                  secondReturnTrajectory.value =
                      secondReturnTrajectorySet.value.get(
                          AllianceFlipUtil.apply(
                                  RobotState.getInstance().getTrajectorySetpoint().getTranslation())
                              .nearest(new ArrayList<>(secondReturnTrajectorySet.value.keySet())));
                  returnTimer.restart();
                }),
            followTrajectory(drive, () -> secondReturnTrajectory.value));
    Command secondIntakeFeed =
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
                                    secondReturnTrajectory.value.getDuration() - 1.5)
                                && (RobotState.getInstance().getTrajectorySetpoint().getY()
                                        > FieldConstants.Stage.ampLeg.getY()
                                    || xCrossed(stageAimX, false)))
                    .andThen(
                        Commands.parallel(aim(drive), superstructure.aimWithCompensation(0.0))));

    // Full command
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
                    followTrajectory(drive, firstIntakeTrajectory)
                        .until(() -> rollers.isTouchingNote() || cancelFirstIntake.getAsBoolean()),
                    Commands.runOnce(
                        () -> {
                          // Select return trajectory
                          firstReturnTrajectory.value =
                              thinking_firstIntakeReturn.get(
                                  AllianceFlipUtil.apply(
                                          RobotState.getInstance()
                                              .getTrajectorySetpoint()
                                              .getTranslation())
                                      .nearest(
                                          new ArrayList<>(thinking_firstIntakeReturn.keySet())));
                          returnTimer.restart();
                        }),
                    followTrajectory(drive, () -> firstReturnTrajectory.value),
                    isInfinite ? secondIntakeDrive.repeatedly() : secondIntakeDrive)
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
                        isInfinite ? secondIntakeFeed.repeatedly() : secondIntakeFeed))
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
                  followTrajectory(drive, spike3ToFirstIntake),
                  followTrajectory(drive, spike2ToFirstIntake),
                  () ->
                      responses
                          .get()
                          .get(1)
                          .equals(AutoQuestionResponse.THREE) // Scores three spikes
                  )
              .andThen(thinkingOnYourFeet(false, false, () -> false, () -> false)));
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
    var startToCenterline = new HolonomicTrajectory("CA_startToCenterline");
    var centerlineToSpikes = new HolonomicTrajectory("CA_centerlineToSpikes");

    Timer autoTimer = new Timer();
    Timer remainingSpikesTimer = new Timer();
    return Commands.sequence(
        Commands.runOnce(autoTimer::restart),
        resetPose(DriveTrajectories.startingAmp),

        // Preload and spike 2
        followTrajectory(drive, startToCenterline)
            .deadlineWith(
                Commands.waitSeconds(0.85)
                    .andThen(rollers.setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED))
                    .deadlineWith(
                        Commands.parallel(
                            aim(drive), superstructure.aimWithCompensation(1.0)))
                    .withTimeout(2.6),
                flywheels.shootCommand()),

        // Thinking-on-your-feet
        Commands.either(
            thinkingOnYourFeet(
                false, true, () -> autoTimer.hasElapsed(4.4), () -> autoTimer.hasElapsed(8.7)),
            thinkingOnYourFeet(true, false, () -> false, () -> false),
            () -> responses.get().get(0).equals(AutoQuestionResponse.YES)),

        // Shoot remaining spikes
        Commands.runOnce(() -> remainingSpikesTimer.restart()),
        followTrajectory(drive, centerlineToSpikes)
            .andThen(Commands.waitSeconds(0.3))
            .deadlineWith(
                rollers.setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED),
                aim(drive),
                superstructure.setGoalCommand(Superstructure.Goal.AIM),
                flywheels.shootCommand()));
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
                    resetPose(DriveTrajectories.startingAmpEdge),
                    followTrajectory(drive, grabCenterline4),
                    followTrajectory(drive, grabCenterline3),
                    followTrajectory(drive, grabCenterline2),
                    followTrajectory(drive, grabEjected))
                .alongWith(
                    Commands.sequence(
                            Commands.waitSeconds(1.65),
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
                            rollers
                                .setGoalCommand(Rollers.Goal.FLOOR_INTAKE)
                                .withTimeout(grabEjected.getDuration() - 0.5),

                            // Score ejected note
                            Commands.waitSeconds(0.55)
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
                    intake(rollers)
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
                                        .deadlineWith(intake(rollers))),

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
                                        .deadlineWith(intake(rollers))),

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

  public Command davisInspirationalAuto() {
    HolonomicTrajectory leaveFromSource = new HolonomicTrajectory("inspirational_leaveFromSource");
    HolonomicTrajectory leaveFromCenter = new HolonomicTrajectory("inspirational_leaveFromCenter");
    HolonomicTrajectory leaveFromAmp = new HolonomicTrajectory("inspirational_leaveFromAmp");
    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            Commands.parallel(
                Commands.sequence(
                    waitUntilXCrossed(FieldConstants.startingLineX, true),
                    Commands.runOnce(
                        () -> System.out.println("Crossed starting line at " + autoTimer.get()))),
                Commands.select(
                        Map.of(
                            AutoQuestionResponse.SOURCE,
                            resetPose(DriveTrajectories.startingSourceSubwoofer),
                            AutoQuestionResponse.CENTER,
                            resetPose(DriveTrajectories.startingCenter),
                            AutoQuestionResponse.AMP,
                            resetPose(DriveTrajectories.startingAmpSubwoofer)),
                        () -> responses.get().get(0) // Starting location
                        )
                    .andThen(
                        // Shoot preload in one second
                        Commands.waitSeconds(1.0 - shootTimeoutSecs.get())
                            .andThen(feed(rollers))
                            .deadlineWith(
                                flywheels.shootCommand(), superstructure.aimWithCompensation(0)),

                        // Wait time
                        Commands.select(
                                Map.of(
                                    AutoQuestionResponse.IMMEDIATELY,
                                    Commands.none(),
                                    AutoQuestionResponse.SIX_SECONDS,
                                    Commands.waitSeconds(5.0),
                                    AutoQuestionResponse.LAST_SECOND,
                                    Commands.select(
                                        Map.of(
                                            AutoQuestionResponse.SOURCE,
                                            Commands.waitSeconds(
                                                13.5 - leaveFromSource.getDuration()),
                                            AutoQuestionResponse.CENTER,
                                            Commands.waitSeconds(
                                                13.5 - leaveFromCenter.getDuration()),
                                            AutoQuestionResponse.AMP,
                                            Commands.waitSeconds(
                                                13.5 - leaveFromAmp.getDuration())),
                                        () -> responses.get().get(0))),
                                () -> responses.get().get(2))
                            .andThen(
                                Commands.select(
                                    Map.of(
                                        AutoQuestionResponse.SOURCE,
                                        followTrajectory(
                                            drive,
                                            new HolonomicTrajectory(
                                                "inspirational_leaveFromSource")),
                                        AutoQuestionResponse.CENTER,
                                        followTrajectory(
                                            drive,
                                            new HolonomicTrajectory(
                                                "inspirational_leaveFromCenter")),
                                        AutoQuestionResponse.AMP,
                                        followTrajectory(
                                            drive,
                                            new HolonomicTrajectory("inspirational_leaveFromAmp"))),
                                    () -> responses.get().get(0)))
                            .onlyIf(() -> responses.get().get(1) == AutoQuestionResponse.YES))));
  }
}
