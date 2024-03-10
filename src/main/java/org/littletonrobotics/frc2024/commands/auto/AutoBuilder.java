// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
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
  final double spikeIntakeDelay = 0.05;
  final double aimDelay = 0.32;

  public AutoBuilder(
      Drive drive, Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.flywheels = flywheels;
    this.rollers = rollers;
  }

  /** Shared trajectories */
  private final HolonomicTrajectory sourceSideStartToSpike0 =
      new HolonomicTrajectory("sourceSpike_grabSpike0");

  private final HolonomicTrajectory centerSideStartToSpike0 =
      new HolonomicTrajectory("centerSpike_grabSpike0");
  private final HolonomicTrajectory centerSideStartToSpike2 =
      new HolonomicTrajectory("centerSpike_grabSpike2");
  private final HolonomicTrajectory ampSideStartToSpike2 =
      new HolonomicTrajectory("ampSpike_grabSpike2");

  /** Between spikes */
  private final HolonomicTrajectory spike0ToSpike1 =
      new HolonomicTrajectory("sourceToAmp_grabSpike1");

  private final HolonomicTrajectory spike1ToSpike2 =
      new HolonomicTrajectory("sourceToAmp_grabSpike2");
  private final HolonomicTrajectory spike2ToSpike1 =
      new HolonomicTrajectory("ampToSource_grabSpike1");
  private final HolonomicTrajectory spike1ToSpike0 =
      new HolonomicTrajectory("ampToSource_grabSpike0");

  /** Spike (0, 2) to centerline 2 and centerline escape */
  private final HolonomicTrajectory spike2ToCenterline2 =
      new HolonomicTrajectory("spike2_grabCenterline2");

  private final HolonomicTrajectory spike0ToCenterline2 =
      new HolonomicTrajectory("spike0_grabCenterline2");
  private final HolonomicTrajectory spike2Escape = new HolonomicTrajectory("spike2Escape");
  private final HolonomicTrajectory spike0Escape = new HolonomicTrajectory("spike0Escape");
  private final HolonomicTrajectory centerline2AroundStageEscape =
      new HolonomicTrajectory("centerline2AroundStageEscape");
  private final HolonomicTrajectory centerline2UnderStageEscape =
      new HolonomicTrajectory("centerline2UnderStageEscape");

  private Command preloadToFirstSpike(HolonomicTrajectory trajectoryFromFace) {
    return Commands.sequence(
            resetPose(
                new Pose2d(
                    trajectoryFromFace.getStartPose().getTranslation(),
                    Rotation2d.fromDegrees(180.0))),
            aim(drive).withTimeout(preloadDelay),
            followTrajectory(drive, trajectoryFromFace),
            aim(drive).withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()))
        .alongWith(
            Commands.sequence(
                // Shoot preload
                Commands.waitUntil(flywheels::atGoal)
                    .andThen(feed(rollers))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

                // Intake first spike
                intake(superstructure, rollers)
                    .withTimeout(trajectoryFromFace.getDuration() + spikeIntakeDelay),

                // Shoot first spike
                Commands.waitSeconds(aimDelay)
                    .andThen(feed(rollers))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))))
        .deadlineWith(flywheels.shootCommand());
  }

  private Command firstSpikeToThirdSpike(boolean endsOnSpike2) {
    HolonomicTrajectory firstTrajectory = endsOnSpike2 ? spike0ToSpike1 : spike2ToSpike1;
    HolonomicTrajectory secondTrajectory = endsOnSpike2 ? spike1ToSpike2 : spike1ToSpike0;
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
                    .andThen(feed(rollers))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

                // Intake spike 2
                intake(superstructure, rollers)
                    .withTimeout(secondTrajectory.getDuration() + spikeIntakeDelay),

                // Shoot spike 2
                Commands.waitSeconds(aimDelay)
                    .andThen(feed(rollers))
                    .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM))))
        .deadlineWith(flywheels.shootCommand());
  }

  private Command thirdSpikeToCenterline2(boolean endsOnSpike2) {
    Timer autoTimer = new Timer();
    if (endsOnSpike2) {
      return Commands.runOnce(autoTimer::restart)
          .andThen(
              // Trajectory
              followTrajectory(drive, spike2ToCenterline2)

                  // Sequence superstructure and rollers
                  .alongWith(
                      Commands.sequence(
                          // Intake centerline 2
                          intake(superstructure, rollers)
                              .withTimeout(spike2ToCenterline2.getDuration() - 2.0),

                          // Shoot centerline 2
                          Commands.waitUntil(
                                  () ->
                                      autoTimer.hasElapsed(
                                          spike2ToCenterline2.getDuration()
                                              + shootTimeoutSecs.get() / 2.0))
                              .andThen(feed(rollers))
                              .deadlineWith(
                                  superstructure.setGoalCommand(Superstructure.Goal.AIM)))))
          // Run flywheels
          .deadlineWith(flywheels.shootCommand());
    } else {
      return Commands.runOnce(autoTimer::restart)
          .andThen(
              // Trajectory
              followTrajectory(drive, spike0ToCenterline2)

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
                                  () ->
                                      autoTimer.hasElapsed(
                                          spike0ToCenterline2.getDuration()
                                              - shootTimeoutSecs.get() / 2.0))
                              .andThen(feed(rollers))
                              .deadlineWith(
                                  waitUntilXCrossed(FieldConstants.Stage.center.getX(), false)
                                      .andThen(
                                          superstructure.setGoalCommand(
                                              Superstructure.Goal.AIM))))))
          // Run flywheels
          .deadlineWith(flywheels.shootCommand());
    }
  }

  private Command escapeToCenterlineFromSpike(boolean endsOnSpike2, BooleanSupplier fiveNote) {
    return Commands.either(
        // Five Note escape
        Commands.either(
            followTrajectory(drive, centerline2AroundStageEscape),
            followTrajectory(drive, centerline2UnderStageEscape),
            () -> endsOnSpike2),
        // Normal escape
        Commands.either(
            followTrajectory(drive, spike2Escape),
            followTrajectory(drive, spike0Escape),
            () -> endsOnSpike2),
        fiveNote);
  }

  public Command buildSpikeAuto(
      SpikeAutoSetup spikeAutoSetup, BooleanSupplier fiveNote, BooleanSupplier escape) {
    Command preloadWithFirstSpike =
        switch (spikeAutoSetup) {
          case SOURCE -> preloadToFirstSpike(sourceSideStartToSpike0);
          case CENTER_SOURCE_TO_AMP -> preloadToFirstSpike(centerSideStartToSpike0);
          case CENTER_AMP_TO_SOURCE -> preloadToFirstSpike(centerSideStartToSpike2);
          case AMP -> preloadToFirstSpike(ampSideStartToSpike2);
        };
    boolean endsOnSpike2 =
        spikeAutoSetup == SpikeAutoSetup.SOURCE
            || spikeAutoSetup == SpikeAutoSetup.CENTER_SOURCE_TO_AMP;
    Command firstSpikeToThirdSpike = firstSpikeToThirdSpike(endsOnSpike2);
    return preloadWithFirstSpike.andThen(
        firstSpikeToThirdSpike,
        // Fifth note
        Commands.either(thirdSpikeToCenterline2(endsOnSpike2), Commands.none(), fiveNote),
        // Escape to centerline at end
        Commands.either(
            escapeToCenterlineFromSpike(endsOnSpike2, fiveNote), Commands.none(), escape));
  }

  public enum SpikeAutoSetup {
    SOURCE,
    CENTER_SOURCE_TO_AMP,
    CENTER_AMP_TO_SOURCE,
    AMP
  }

  public Command N5_S01_C2_S2() {
    var grabCenterline2 = new HolonomicTrajectory("N5_S01_C2_S2_grabCenterline2");
    var grabSpike2 = new HolonomicTrajectory("N5_S01_C2_S2_grabSpike2");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            Commands.sequence(
                resetPose(DriveTrajectories.startingLineSpike0),
                Commands.startEnd(
                        () ->
                            drive.setHeadingGoal(
                                () ->
                                    RobotState.getInstance().getAimingParameters().driveHeading()),
                        drive::clearHeadingGoal)
                    .withTimeout(preloadDelay),
                followTrajectory(drive, sourceSideStartToSpike0),
                Commands.startEnd(
                        () ->
                            drive.setHeadingGoal(
                                () ->
                                    RobotState.getInstance().getAimingParameters().driveHeading()),
                        drive::clearHeadingGoal)
                    .withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()),
                followTrajectory(drive, spike0ToSpike1),
                Commands.startEnd(
                        () ->
                            drive.setHeadingGoal(
                                () ->
                                    RobotState.getInstance().getAimingParameters().driveHeading()),
                        drive::clearHeadingGoal)
                    .withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()),
                followTrajectory(drive, grabCenterline2),
                Commands.startEnd(
                        () ->
                            drive.setHeadingGoal(
                                () ->
                                    RobotState.getInstance().getAimingParameters().driveHeading()),
                        drive::clearHeadingGoal)
                    .withTimeout(spikeIntakeDelay + aimDelay + shootTimeoutSecs.get()),
                followTrajectory(drive, grabSpike2)));
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
                            .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

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
                            .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

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
                                waitUntilXCrossed(FieldConstants.Stage.center.getX(), false)
                                    .andThen(
                                        superstructure.setGoalCommand(Superstructure.Goal.AIM))),

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
                                waitUntilXCrossed(FieldConstants.Stage.center.getX(), false)
                                    .andThen(
                                        superstructure.setGoalCommand(Superstructure.Goal.AIM)))))

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
                            .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

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
                            .setGoalCommand(Superstructure.Goal.AIM)
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
                            .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

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
                            .deadlineWith(superstructure.setGoalCommand(Superstructure.Goal.AIM)),

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
                                waitUntilXCrossed(FieldConstants.Stage.center.getX() + 0.1, false)
                                    .andThen(
                                        superstructure.setGoalCommand(Superstructure.Goal.AIM)))))

                // Run flywheels
                .deadlineWith(flywheels.shootCommand()));
  }
}
