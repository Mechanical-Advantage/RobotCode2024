// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.littletonrobotics.frc2024.AutoSelector.*;
import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.DriveTrajectories;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;

@RequiredArgsConstructor
public class AutoBuilder {
  private static final double preloadDelay = 1.0;
  private static final double spikeIntakeDelay = 0.35;
  private static final double aimDelay = 0.45;
  private static final double stageAimX = FieldConstants.Stage.center.getX() - 0.3;

  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;
  private final Supplier<List<AutoQuestionResponse>> responses;

  // Shared trajectories
  // Starting locations to spikes
  private final HolonomicTrajectory centerStartToSpike0 =
      new HolonomicTrajectory("centerStartToSpike0");
  private final HolonomicTrajectory sourceStartToSpike0 =
      new HolonomicTrajectory("sourceStartToSpike0");
  private final HolonomicTrajectory ampStartToSpike2 = new HolonomicTrajectory("ampStartToSpike2");

  // Between spikes
  private final HolonomicTrajectory spike0ToSpike1 = new HolonomicTrajectory("spike0ToSpike1");
  private final HolonomicTrajectory spike1ToSpike2 = new HolonomicTrajectory("spike1ToSpike2");
  private final HolonomicTrajectory spike2ToSpike1 = new HolonomicTrajectory("spike2ToSpike1");
  private final HolonomicTrajectory spike1ToSpike0 = new HolonomicTrajectory("spike1ToSpike0");

  // Drive to center
  private final HolonomicTrajectory spike2ToCenter = new HolonomicTrajectory("spike2ToCenter");
  private final HolonomicTrajectory spike0ToCenter = new HolonomicTrajectory("spike0ToCenter");
  private final HolonomicTrajectory wingLeftShotToCenter =
      new HolonomicTrajectory("wingLeftShotToCenter");
  private final HolonomicTrajectory wingRightShotToCenter =
      new HolonomicTrajectory("wingRightShotToCenter");
  private final HolonomicTrajectory underStageShotToCenter =
      new HolonomicTrajectory("underStageShotToCenter");

  public Command spike4(boolean driveToCenter) {
    return select(
            Map.of(
                AutoQuestionResponse.SOURCE,
                preloadToFirstSpike(DriveTrajectories.startingLineSpike0, sourceStartToSpike0)
                    .andThen(firstSpikeToThirdSpike(true)),
                AutoQuestionResponse.CENTER,
                preloadToFirstSpike(DriveTrajectories.startingLineSpike1, centerStartToSpike0)
                    .andThen(firstSpikeToThirdSpike(true)),
                AutoQuestionResponse.AMP,
                preloadToFirstSpike(DriveTrajectories.startingLineSpike2, ampStartToSpike2)
                    .andThen(firstSpikeToThirdSpike(false))),
            () -> responses.get().get(0))
        .andThen(
            either(
                    followTrajectory(drive, spike0ToCenter),
                    followTrajectory(drive, spike2ToCenter),
                    () -> responses.get().get(0).equals(AutoQuestionResponse.AMP))
                .onlyIf(() -> driveToCenter));
  }

  public Command spike5() {
    Supplier<CenterlineNote> centerlineNote =
        () -> CenterlineNote.valueOf(responses.get().get(1).toString());
    Supplier<CenterlineShot> centerlineShot =
        () ->
            responses.get().get(0).equals(AutoQuestionResponse.AMP)
                ? centerlineNote.get().fromSpike0ToCenterlineShot
                : centerlineNote.get().fromSpike2ToCenterlineShot;
    return spike4(false)
        .andThen(
            select(
                Map.of(
                    CenterlineNote.ZERO,
                    spikeToCenterline(CenterlineNote.ZERO),
                    CenterlineNote.ONE,
                    spikeToCenterline(CenterlineNote.ONE),
                    CenterlineNote.TWO,
                    spikeToCenterline(CenterlineNote.TWO),
                    CenterlineNote.THREE,
                    spikeToCenterline(CenterlineNote.THREE),
                    CenterlineNote.FOUR,
                    spikeToCenterline(CenterlineNote.FOUR)),
                centerlineNote),
            select(
                Map.of(
                    CenterlineShot.WING_LEFT,
                    followTrajectory(drive, wingLeftShotToCenter),
                    CenterlineShot.UNDER_STAGE,
                    followTrajectory(drive, underStageShotToCenter),
                    CenterlineShot.WING_RIGHT,
                    followTrajectory(drive, wingRightShotToCenter)),
                centerlineShot));
  }

  private Command preloadToFirstSpike(Pose2d startPose, HolonomicTrajectory preloadToFirstSpike) {
    return Commands.sequence(
            resetPose(startPose),
            aim(drive).withTimeout(preloadDelay),
            followTrajectory(drive, preloadToFirstSpike),
            aim(drive).withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()))
        .alongWith(
            Commands.sequence(
                // Shoot preload
                Commands.waitUntil(flywheels::atGoal)
                    .andThen(feed(rollers))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))
                    .withTimeout(preloadDelay),

                // Intake first spike
                intake(superstructure, rollers)
                    .withTimeout(preloadToFirstSpike.getDuration() + spikeIntakeDelay),

                // Shoot first spike
                Commands.waitSeconds(aimDelay)
                    .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))))
        .deadlineWith(flywheels.shootCommand());
  }

  private Command firstSpikeToThirdSpike(boolean startWithSpike0) {
    HolonomicTrajectory firstTrajectory = startWithSpike0 ? spike0ToSpike1 : spike2ToSpike1;
    HolonomicTrajectory secondTrajectory = startWithSpike0 ? spike1ToSpike2 : spike1ToSpike0;
    return Commands.sequence(
            followTrajectory(drive, firstTrajectory),
            aim(drive).withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()),
            followTrajectory(drive, secondTrajectory),
            aim(drive).withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()))
        .alongWith(
            Commands.sequence(
                // Intake spike 1
                intake(superstructure, rollers)
                    .withTimeout(firstTrajectory.getDuration() + spikeIntakeDelay),

                // Shoot spike 1
                Commands.waitSeconds(aimDelay)
                    .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

                // Intake spike 2
                intake(superstructure, rollers)
                    .withTimeout(secondTrajectory.getDuration() + spikeIntakeDelay),

                // Shoot spike 2
                Commands.waitSeconds(aimDelay)
                    .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))))
        .deadlineWith(flywheels.shootCommand());
  }

  private Command spikeToCenterline(CenterlineNote centerlineNote) {
    return either(
        spikeToCenterline(
            true,
            centerlineNote.index,
            centerlineNote.fromSpike0ToCenterlineShot.equals(CenterlineShot.UNDER_STAGE)),
        spikeToCenterline(
            false,
            centerlineNote.index,
            centerlineNote.fromSpike2ToCenterlineShot.equals(CenterlineShot.UNDER_STAGE)),
        // Starts at amp
        () -> responses.get().get(0).equals(AutoQuestionResponse.AMP));
  }

  private Command spikeToCenterline(
      boolean startsWithSpike0, int centerlineIndex, boolean underStageShot) {
    HolonomicTrajectory trajectory =
        new HolonomicTrajectory(
            "spike" + (startsWithSpike0 ? 0 : 2) + "ToCenterline" + centerlineIndex);
    Timer timer = new Timer();
    return runOnce(timer::restart)
        .andThen(
            // Follow trajectory and aim
            followTrajectory(drive, trajectory)
                .andThen(aim(drive).withTimeout(shootTimeoutSecs.get()))
                .alongWith(
                    underStageShot
                        ?
                        // Does not shoot from under stage
                        sequence(
                            // Intake until aiming
                            intake(superstructure, rollers)
                                .withTimeout(trajectory.getDuration() - aimDelay * 2.0),

                            // Shoot
                            waitSeconds(aimDelay * 2.0)
                                .andThen(feed(rollers))
                                .deadlineWith(superstructure.aimWithCompensation(0.0)))
                        :

                        // Does shoot from under stage
                        sequence(
                            // Intake centerline note
                            waitUntilXCrossed(
                                    FieldConstants.wingX
                                        + DriveConstants.driveConfig.bumperWidthX()
                                        + 0.25,
                                    true)
                                .andThen(
                                    waitUntilXCrossed(
                                            FieldConstants.wingX
                                                + DriveConstants.driveConfig.bumperWidthX()
                                                + 0.2,
                                            false)
                                        .deadlineWith(intake(superstructure, rollers))),

                            // Shoot centerline note
                            waitUntil(() -> timer.hasElapsed(trajectory.getDuration()))
                                .andThen(feed(rollers))
                                .deadlineWith(
                                    waitUntilXCrossed(stageAimX, false)
                                        .andThen(
                                            superstructure.setGoalCommand(
                                                Superstructure.Goal.AIM)))))
                .deadlineWith(flywheels.shootCommand()));
  }

  @RequiredArgsConstructor
  private enum CenterlineNote {
    ZERO(0, CenterlineShot.WING_RIGHT, CenterlineShot.WING_RIGHT),
    ONE(1, CenterlineShot.UNDER_STAGE, CenterlineShot.WING_RIGHT),
    TWO(2, CenterlineShot.WING_LEFT, CenterlineShot.UNDER_STAGE),
    THREE(3, CenterlineShot.WING_LEFT, CenterlineShot.WING_LEFT),
    FOUR(4, CenterlineShot.WING_LEFT, CenterlineShot.WING_LEFT);

    private final int index;
    private final CenterlineShot fromSpike2ToCenterlineShot;
    private final CenterlineShot fromSpike0ToCenterlineShot;
  }

  private enum CenterlineShot {
    WING_LEFT,
    UNDER_STAGE,
    WING_RIGHT
  }

  public Command davisEthicalAuto() {
    var grabCenterline0 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline0");
    var grabCenterline1 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline1");
    var grabCenterline2 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline2");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive sequence
            Commands.sequence(
                    resetPose(DriveTrajectories.startingLineSpike0),
                    Commands.startEnd(
                            () ->
                                drive.setHeadingGoal(
                                    () ->
                                        RobotState.getInstance()
                                            .getAimingParameters()
                                            .driveHeading()),
                            drive::clearHeadingGoal)
                        .withTimeout(preloadDelay),
                    followTrajectory(drive, grabCenterline0),
                    followTrajectory(drive, grabCenterline1),
                    followTrajectory(drive, grabCenterline2))

                // Superstructure & rollers sequence
                .alongWith(
                    Commands.sequence(
                        // Shoot preload
                        Commands.waitUntil(flywheels::atGoal)
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .until(() -> autoTimer.hasElapsed(preloadDelay)))
                            .deadlineWith(superstructure.aimWithCompensation(0.0)),

                        // Intake centerline 0
                        waitUntilXCrossed(FieldConstants.wingX + 0.05, true)
                            .andThen(waitUntilXCrossed(FieldConstants.wingX, false))
                            .deadlineWith(
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE)),

                        // Shoot centerline 0
                        Commands.waitUntil(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabCenterline0.getDuration()
                                            - shootTimeoutSecs.get()))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(superstructure.aimWithCompensation(0.0)),

                        // Intake centerline 1
                        waitUntilXCrossed(
                                FieldConstants.wingX
                                    + DriveConstants.driveConfig.bumperWidthX()
                                    + 0.3,
                                true)
                            .andThen(
                                waitUntilXCrossed(
                                    FieldConstants.wingX
                                        + DriveConstants.driveConfig.bumperWidthX()
                                        + 0.25,
                                    false))
                            .deadlineWith(
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE)),

                        // Shoot centerline 1
                        Commands.waitUntil(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabCenterline0.getDuration()
                                            + grabCenterline1.getDuration()
                                            - shootTimeoutSecs.get()))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(
                                waitUntilXCrossed(stageAimX, false)
                                    .andThen(superstructure.aimWithCompensation(0.0))),

                        // Intake centerline 2
                        waitUntilXCrossed(
                                FieldConstants.wingX
                                    + DriveConstants.driveConfig.bumperWidthX()
                                    + 0.3,
                                true)
                            .andThen(
                                waitUntilXCrossed(
                                        FieldConstants.wingX
                                            + DriveConstants.driveConfig.bumperWidthX()
                                            + 0.25,
                                        false)
                                    .deadlineWith(
                                        superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                        rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))),

                        // Shoot centerline 2
                        Commands.waitUntil(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabCenterline0.getDuration()
                                            + grabCenterline1.getDuration()
                                            + grabCenterline2.getDuration()
                                            - shootTimeoutSecs.get()))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(
                                waitUntilXCrossed(stageAimX, false)
                                    .andThen(superstructure.aimWithCompensation(0.0)))))

                // Run flywheels
                .deadlineWith(flywheels.shootCommand()));
  }

  public Command davisAlternativeAuto() {
    var grabSpike = new HolonomicTrajectory("davisAlternativeAuto_grabSpike");
    var grabCenterline4 = new HolonomicTrajectory("davisAlternativeAuto_grabCenterline4");
    var grabCenterline3 = new HolonomicTrajectory("davisAlternativeAuto_grabCenterline3");
    var grabCenterline2 = new HolonomicTrajectory("davisAlternativeAuto_grabCenterline2");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive sequence
            Commands.sequence(
                    resetPose(DriveTrajectories.startingLineSpike2),
                    Commands.startEnd(
                            () ->
                                drive.setHeadingGoal(
                                    () ->
                                        RobotState.getInstance()
                                            .getAimingParameters()
                                            .driveHeading()),
                            drive::clearHeadingGoal)
                        .withTimeout(preloadDelay),
                    followTrajectory(drive, grabSpike),
                    Commands.waitSeconds(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()),
                    followTrajectory(drive, grabCenterline4),
                    followTrajectory(drive, grabCenterline3),
                    followTrajectory(drive, grabCenterline2))

                // Superstructure & rollers sequence
                .alongWith(
                    Commands.sequence(
                        // Shoot preload
                        Commands.waitUntil(flywheels::atGoal)
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .until(() -> autoTimer.hasElapsed(preloadDelay)))
                            .deadlineWith(superstructure.aimWithCompensation(0.0)),

                        // Intake spike
                        Commands.parallel(
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE))
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay + grabSpike.getDuration() + spikeIntakeDelay)),

                        // Shoot spike
                        superstructure
                            .aimWithCompensation(0.0)
                            .alongWith(
                                Commands.waitSeconds(aimDelay)
                                    .andThen(rollers.setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)))
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabSpike.getDuration()
                                            + spikeIntakeDelay
                                            + aimDelay
                                            + shootTimeoutSecs.get())),

                        // Intake centerline 4
                        waitUntilXCrossed(FieldConstants.wingX + 0.85, true)
                            .andThen(waitUntilXCrossed(FieldConstants.wingX + 0.8, false))
                            .deadlineWith(
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE)),

                        // Shoot centerline 4
                        Commands.waitUntil(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabSpike.getDuration()
                                            + spikeIntakeDelay
                                            + aimDelay
                                            + shootTimeoutSecs.get()
                                            + grabCenterline4.getDuration()
                                            - shootTimeoutSecs.get() / 2))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(superstructure.aimWithCompensation(0.0)),

                        // Intake centerline 3
                        waitUntilXCrossed(FieldConstants.wingX + 0.85, true)
                            .andThen(waitUntilXCrossed(FieldConstants.wingX + 0.8, false))
                            .deadlineWith(
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE)),

                        // Shoot centerline 3
                        Commands.waitUntil(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabSpike.getDuration()
                                            + spikeIntakeDelay
                                            + aimDelay
                                            + shootTimeoutSecs.get()
                                            + grabCenterline4.getDuration()
                                            + grabCenterline3.getDuration()
                                            - shootTimeoutSecs.get() / 2))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(superstructure.aimWithCompensation(0.0)),

                        // Intake centerline 2
                        waitUntilXCrossed(
                                FieldConstants.wingX
                                    + DriveConstants.driveConfig.bumperWidthX()
                                    + 0.3,
                                true)
                            .andThen(
                                waitUntilXCrossed(
                                    FieldConstants.wingX
                                        + DriveConstants.driveConfig.bumperWidthX()
                                        + 0.25,
                                    false))
                            .deadlineWith(
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE)),

                        // Shoot centerline 2
                        Commands.waitUntil(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabSpike.getDuration()
                                            + spikeIntakeDelay
                                            + aimDelay
                                            + shootTimeoutSecs.get()
                                            + grabCenterline4.getDuration()
                                            + grabCenterline3.getDuration()
                                            + grabCenterline2.getDuration()
                                            - shootTimeoutSecs.get()))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(
                                waitUntilXCrossed(stageAimX, false)
                                    .andThen(superstructure.aimWithCompensation(0.0)))))

                // Run flywheels
                .deadlineWith(flywheels.shootCommand()));
  }

  public Command davisSuperAlternativeAuto() {
    var grabCenterline3 = new HolonomicTrajectory("spike1ToCenterline3");
    var grabCenterline2 = new HolonomicTrajectory("centerline3ToCenterline2");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            Commands.sequence(
                    // Drive Sequence
                    resetPose(DriveTrajectories.startingLineSpike2),
                    aim(drive).withTimeout(preloadDelay),
                    followTrajectory(drive, ampStartToSpike2),
                    aim(drive).withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()),
                    followTrajectory(drive, spike2ToSpike1),
                    aim(drive).withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()),
                    followTrajectory(drive, grabCenterline3),
                    followTrajectory(drive, grabCenterline2),
                    Commands.waitSeconds(shootTimeoutSecs.get()),
                    aim(drive).withTimeout(shootTimeoutSecs.get() / 2.0))
                .alongWith(
                    // Superstructure and rollers sequence
                    Commands.sequence(
                            // Shoot preload
                            Commands.waitUntil(flywheels::atGoal)
                                .andThen(feed(rollers))
                                .deadlineWith(
                                    superstructure.setGoalCommand(Superstructure.Goal.AIM))
                                .withTimeout(preloadDelay),

                            // Intake spike 2
                            intake(superstructure, rollers)
                                .withTimeout(ampStartToSpike2.getDuration() + spikeIntakeDelay),

                            // Shoot spike 2
                            Commands.waitSeconds(aimDelay)
                                .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                                .deadlineWith(superstructure.aimWithCompensation(0.0)),

                            // Intake spike 1
                            intake(superstructure, rollers)
                                .withTimeout(spike2ToSpike1.getDuration() + spikeIntakeDelay),

                            // Shoot spike 1
                            Commands.waitSeconds(aimDelay)
                                .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                                .deadlineWith(superstructure.aimWithCompensation(0.0)),

                            // Intake centerline 3
                            intake(superstructure, rollers)
                                .withTimeout(grabCenterline3.getDuration() - 1.0),

                            // Shoot centerline 3
                            Commands.waitUntil(
                                    () ->
                                        autoTimer.hasElapsed(
                                            preloadDelay
                                                + ampStartToSpike2.getDuration()
                                                + spike2ToSpike1.getDuration()
                                                + (spikeIntakeDelay
                                                        + aimDelay
                                                        + shootTimeoutSecs.get())
                                                    * 2.0
                                                + grabCenterline3.getDuration()
                                                - shootTimeoutSecs.get() / 2.0))
                                .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                                .deadlineWith(superstructure.aimWithCompensation(0.0)),

                            // Intake centerline 2
                            waitUntilXCrossed(
                                    FieldConstants.wingX
                                        + DriveConstants.driveConfig.bumperWidthX()
                                        + 0.3,
                                    true)
                                .andThen(
                                    waitUntilXCrossed(
                                            FieldConstants.wingX
                                                + DriveConstants.driveConfig.bumperWidthX()
                                                + 0.25,
                                            false)
                                        .deadlineWith(intake(superstructure, rollers))),

                            // Shoot centerline 2
                            Commands.waitUntil(
                                    () ->
                                        autoTimer.hasElapsed(
                                            preloadDelay
                                                + ampStartToSpike2.getDuration()
                                                + spike2ToSpike1.getDuration()
                                                + (spikeIntakeDelay
                                                        + aimDelay
                                                        + shootTimeoutSecs.get())
                                                    * 2.0
                                                + grabCenterline3.getDuration()
                                                + grabCenterline2.getDuration()
                                                - shootTimeoutSecs.get() / 2.0))
                                .andThen(feed(rollers))
                                .deadlineWith(
                                    waitUntilXCrossed(stageAimX, false)
                                        .andThen(superstructure.aimWithCompensation(0.0))))
                        // Run flywheels
                        .deadlineWith(flywheels.shootCommand())));
  }
}
