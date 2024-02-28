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

  public AutoBuilder(
      Drive drive, Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.flywheels = flywheels;
    this.rollers = rollers;
  }

  public Command davisEthicalAuto() {
    var grabCenterline0 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline0");
    var grabCenterline1 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline1");
    var grabCenterline2 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline2");

    final double preloadDelay = 1.0;

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive sequence
            Commands.sequence(
                    resetPose(DriveTrajectories.startingLinePodium),
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
                                waitUntilXCrossed(FieldConstants.Stage.center.getX() + 0.1, false)
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
                                waitUntilXCrossed(FieldConstants.Stage.center.getX() + 0.1, false)
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

    final double preloadDelay = 1.0;
    final double spikeIntakeDelay = 0.5;
    final double spikeAimDelay = 0.4;

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
                    Commands.waitSeconds(spikeIntakeDelay + spikeAimDelay + shootTimeoutSecs.get()),
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
                                Commands.waitSeconds(spikeAimDelay)
                                    .andThen(rollers.setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)))
                            .until(
                                () ->
                                    autoTimer.hasElapsed(
                                        preloadDelay
                                            + grabSpike.getDuration()
                                            + spikeIntakeDelay
                                            + spikeAimDelay
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
                                            + spikeAimDelay
                                            + shootTimeoutSecs.get()
                                            + grabCenterline4.getDuration()
                                            - shootTimeoutSecs.get()))
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
                                            + spikeAimDelay
                                            + shootTimeoutSecs.get()
                                            + grabCenterline4.getDuration()
                                            + grabCenterline3.getDuration()
                                            - shootTimeoutSecs.get()))
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
                                            + spikeAimDelay
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