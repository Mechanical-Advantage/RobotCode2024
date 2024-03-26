// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.DriveTrajectories;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;

public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;

  final double preloadDelay = 1.0;
  final double spikeIntakeDelay = 0.35;
  final double aimDelay = 0.45;
  final double preloadAimDelay = 0.8;
  final double spikeAimDelay = 0.2;

  final double stageAimX = FieldConstants.Stage.center.getX() - 0.3;

  public AutoBuilder(
      Drive drive, Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.flywheels = flywheels;
    this.rollers = rollers;
  }

  // Shared trajectories
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

  // Spike (0, 2) to centerline 2
  private final HolonomicTrajectory spike2ToCenterline2 =
      new HolonomicTrajectory("spike2ToCenterline2");
  private final HolonomicTrajectory spike0ToCenterline2 =
      new HolonomicTrajectory("spike0ToCenterline2");

  // Drive to center
  private final HolonomicTrajectory spike2ToCenter = new HolonomicTrajectory("spike2ToCenter");
  private final HolonomicTrajectory spike0ToCenter = new HolonomicTrajectory("spike0ToCenter");
  private final HolonomicTrajectory wingShotToCenter = new HolonomicTrajectory("wingShotToCenter");
  private final HolonomicTrajectory underStageShotToCenter =
      new HolonomicTrajectory("underStageShotToCenter");

  private Command preloadToFirstSpike(
      Pose2d startPose,
      HolonomicTrajectory preloadToFirstSpike,
      HolonomicTrajectory nextTrajectory) {
    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .alongWith(
            Commands.sequence(
                    resetPose(startPose),
                    // Aim preload and orient modules for first spike trajectory
                    aim(drive)
                        .withTimeout(preloadAimDelay)
                        .andThen(drive.orientModules(preloadToFirstSpike))
                        .until(() -> timer.hasElapsed(preloadDelay)),
                    followTrajectory(drive, preloadToFirstSpike),

                    // Aim first spike shot and orient modules for next trajectory
                    aim(drive)
                        .withTimeout(spikeAimDelay)
                        .andThen(drive.orientModules(nextTrajectory))
                        .until(
                            () ->
                                timer.hasElapsed(
                                    preloadDelay
                                        + preloadToFirstSpike.getDuration()
                                        + spikeIntakeDelay
                                        + aimDelay
                                        + shootTimeoutSecs.get())))
                .alongWith(
                    Commands.sequence(
                        // Shoot preload
                        Commands.waitUntil(flywheels::atGoal)
                            .andThen(feed(rollers))
                            .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))
                            .until(() -> timer.hasElapsed(preloadDelay)),

                        // Intake first spike
                        intake(superstructure, rollers)
                            .withTimeout(preloadToFirstSpike.getDuration() + spikeIntakeDelay),

                        // Shoot first spike
                        Commands.waitSeconds(aimDelay)
                            .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))))
                .deadlineWith(flywheels.shootCommand()));
  }

  private Command firstSpikeToThirdSpike(
      boolean startWithSpike0, HolonomicTrajectory nextTrajectory) {
    HolonomicTrajectory firstTrajectory = startWithSpike0 ? spike0ToSpike1 : spike2ToSpike1;
    HolonomicTrajectory secondTrajectory = startWithSpike0 ? spike1ToSpike2 : spike1ToSpike0;

    Timer timer = new Timer();
    return Commands.runOnce(timer::restart)
        .alongWith(
            Commands.sequence(
                    followTrajectory(drive, firstTrajectory),
                    // Aim and then orient modules for second trajectory
                    aim(drive)
                        .withTimeout(spikeAimDelay)
                        .andThen(drive.orientModules(secondTrajectory))
                        .until(
                            () ->
                                timer.hasElapsed(
                                    firstTrajectory.getDuration()
                                        + spikeIntakeDelay
                                        + aimDelay
                                        + shootTimeoutSecs.get())),
                    followTrajectory(drive, secondTrajectory),

                    // Aim and then orient modules for next trajectory
                    aim(drive)
                        .withTimeout(spikeAimDelay)
                        .andThen(drive.orientModules(nextTrajectory))
                        .until(
                            () ->
                                timer.hasElapsed(
                                    firstTrajectory.getDuration()
                                        + nextTrajectory.getDuration()
                                        + (spikeIntakeDelay + aimDelay + shootTimeoutSecs.get())
                                            * 2.0)))
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
                .deadlineWith(flywheels.shootCommand()));
  }

  private Command thirdSpikeToCenterline2(boolean startFromSpike2) {
    if (startFromSpike2) {
      return
      // Trajectory
      followTrajectory(drive, spike2ToCenterline2)
          .andThen(aim(drive).withTimeout(shootTimeoutSecs.get()))

          // Sequence superstructure and rollers
          .alongWith(
              Commands.sequence(
                  // Intake centerline 2
                  intake(superstructure, rollers)
                      .withTimeout(spike2ToCenterline2.getDuration() - aimDelay * 2.0),

                  // Shoot centerline 2
                  Commands.waitSeconds(aimDelay * 2.0)
                      .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                      .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))))

          // Run flywheels
          .deadlineWith(flywheels.shootCommand());
    } else {
      Timer timer = new Timer();
      return Commands.runOnce(timer::restart)
          .andThen(
              // Trajectory
              followTrajectory(drive, spike0ToCenterline2)
                  .andThen(aim(drive).withTimeout(shootTimeoutSecs.get()))

                  // Sequence superstructure and rollers
                  .alongWith(
                      Commands.sequence(
                          // Intake centerline 2
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

                          // Shoot centerline 2
                          Commands.waitUntil(
                                  () -> timer.hasElapsed(spike0ToCenterline2.getDuration()))
                              .andThen(feed(rollers).withTimeout(shootTimeoutSecs.get()))
                              .deadlineWith(
                                  waitUntilXCrossed(stageAimX, false)
                                      .andThen(
                                          superstructure.setGoalCommand(
                                              Superstructure.Goal.AIM))))))
          // Run flywheels
          .deadlineWith(flywheels.shootCommand());
    }
  }

  public Command source4() {
    return preloadToFirstSpike(
            DriveTrajectories.startingLineSpike0, sourceStartToSpike0, spike0ToSpike1)
        .andThen(firstSpikeToThirdSpike(true, spike2ToCenter))
        .andThen(followTrajectory(drive, spike2ToCenter));
  }

  public Command source5() {
    return preloadToFirstSpike(
            DriveTrajectories.startingLineSpike0, sourceStartToSpike0, spike0ToSpike1)
        .andThen(firstSpikeToThirdSpike(true, spike2ToCenterline2))
        .andThen(thirdSpikeToCenterline2(true))
        .andThen(followTrajectory(drive, wingShotToCenter));
  }

  public Command center4() {
    return preloadToFirstSpike(
            DriveTrajectories.startingLineSpike1, centerStartToSpike0, spike0ToSpike1)
        .andThen(firstSpikeToThirdSpike(true, spike0ToCenter))
        .andThen(followTrajectory(drive, spike2ToCenter));
  }

  public Command center5() {
    return preloadToFirstSpike(
            DriveTrajectories.startingLineSpike1, centerStartToSpike0, spike0ToSpike1)
        .andThen(firstSpikeToThirdSpike(true, spike0ToCenterline2))
        .andThen(thirdSpikeToCenterline2(true))
        .andThen(followTrajectory(drive, wingShotToCenter));
  }

  public Command amp4() {
    return preloadToFirstSpike(
            DriveTrajectories.startingLineSpike2, ampStartToSpike2, spike2ToSpike1)
        .andThen(firstSpikeToThirdSpike(false, spike2ToCenter))
        .andThen(followTrajectory(drive, spike0ToCenter));
  }

  public Command amp5() {
    return preloadToFirstSpike(
            DriveTrajectories.startingLineSpike2, ampStartToSpike2, spike2ToSpike1)
        .andThen(firstSpikeToThirdSpike(false, spike2ToCenter))
        .andThen(thirdSpikeToCenterline2(false))
        .andThen(followTrajectory(drive, underStageShotToCenter));
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
                    // Aim and then orient modules for preload trajectory
                    aim(drive)
                        .withTimeout(preloadAimDelay)
                        .andThen(drive.orientModules(grabSpike))
                        .until(() -> autoTimer.hasElapsed(preloadDelay)),
                    followTrajectory(drive, grabSpike),
                    aim(drive)
                        .withTimeout(preloadAimDelay)
                        .andThen(drive.orientModules(grabCenterline4))
                        .until(
                            () ->
                                autoTimer.hasElapsed(
                                    preloadDelay
                                        + grabSpike.getDuration()
                                        + aimDelay
                                        + shootTimeoutSecs.get())),
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
                    // Aim and then follow preload trajectory
                    aim(drive)
                        .withTimeout(preloadAimDelay)
                        .andThen(drive.orientModules(ampStartToSpike2))
                        .until(() -> autoTimer.hasElapsed(preloadDelay)),
                    followTrajectory(drive, ampStartToSpike2),
                    // Aim and then follow spike 1 trajectory
                    aim(drive)
                        .withTimeout(spikeAimDelay)
                        .andThen(drive.orientModules(ampStartToSpike2))
                        .until(
                            () ->
                                autoTimer.hasElapsed(
                                    preloadDelay
                                        + ampStartToSpike2.getDuration()
                                        + spikeIntakeDelay
                                        + aimDelay
                                        + shootTimeoutSecs.get())),
                    followTrajectory(drive, spike2ToSpike1),
                    aim(drive)
                        .withTimeout(spikeAimDelay)
                        .andThen(drive.orientModules(grabCenterline3))
                        .until(
                            () ->
                                autoTimer.hasElapsed(
                                    preloadDelay
                                        + ampStartToSpike2.getDuration()
                                        + spike2ToSpike1.getDuration()
                                        + (spikeIntakeDelay + aimDelay + shootTimeoutSecs.get())
                                            * 2.0)),
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
