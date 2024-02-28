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
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;

// import org.littletonrobotics.frc2024.util.NoteVisualizer;

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
    var driveToPodium = new HolonomicTrajectory("davisEthicalAuto_driveToPodium");
    var grabCenterline0 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline0");
    var grabCenterline1 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline1");
    var grabCenterline2 = new HolonomicTrajectory("davisEthicalAuto_grabCenterline2");

    final double initialWait = 0.6; // Delay start to allow flywheels & arm to aim
    final double podiumShootStart = initialWait + 0.6;
    final double podiumWait = 0.3;

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            // Drive sequence
            Commands.sequence(
                    resetPose(driveToPodium),
                    Commands.waitSeconds(initialWait),
                    followTrajectory(drive, driveToPodium),
                    Commands.waitSeconds(podiumWait),
                    followTrajectory(drive, grabCenterline0),
                    followTrajectory(drive, grabCenterline1),
                    followTrajectory(drive, grabCenterline2))

                // Superstructure & rollers sequence
                .alongWith(
                    Commands.sequence(
                        // Intake to shoot for preload and podium
                        Commands.waitSeconds(podiumShootStart)
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED)
                                    .until(
                                        () ->
                                            autoTimer.hasElapsed(
                                                initialWait
                                                    + driveToPodium.getDuration()
                                                    + podiumWait)))
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
                                        initialWait
                                            + driveToPodium.getDuration()
                                            + podiumWait
                                            + grabCenterline0.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
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
                                        initialWait
                                            + driveToPodium.getDuration()
                                            + podiumWait
                                            + grabCenterline0.getDuration()
                                            + grabCenterline1.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(
                                waitUntilXCrossed(FieldConstants.Stage.center.getX() - 0.1, false)
                                    .andThen(
                                        superstructure.setGoalCommand(Superstructure.Goal.INTAKE))),

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
                                        initialWait
                                            + driveToPodium.getDuration()
                                            + podiumWait
                                            + grabCenterline0.getDuration()
                                            + grabCenterline1.getDuration()
                                            + grabCenterline2.getDuration()
                                            - shootTimeoutSecs.get() / 2.0))
                            .andThen(
                                rollers
                                    .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                    .withTimeout(shootTimeoutSecs.get()))
                            .deadlineWith(
                                waitUntilXCrossed(FieldConstants.Stage.center.getX() - 0.1, false)
                                    .andThen(
                                        superstructure.setGoalCommand(Superstructure.Goal.AIM)))))

                // Run flywheels
                .deadlineWith(flywheels.shootCommand()));
  }

  public Command unethicalAuto() {
    var grabAll = new HolonomicTrajectory("unethicalAuto_grabAll");
    var grabPodium = new HolonomicTrajectory("unethicalAuto_grabPodium");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            resetPose(grabAll),
            followTrajectory(drive, grabAll)
                .deadlineWith(
                    Commands.sequence(
                        // Shoot preload
                        waitUntilXCrossed(FieldConstants.wingX, true)
                            .deadlineWith(
                                superstructure.setGoalCommand(Superstructure.Goal.AIM),
                                flywheels.shootCommand(),
                                Commands.waitUntil(flywheels::atGoal)
                                    .andThen(
                                        rollers
                                            .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                                            .withTimeout(shootTimeoutSecs.get()))),

                        // Intake and eject
                        rollers
                            .setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED)
                            .alongWith(
                                flywheels.ejectCommand(),
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE))
                            .until(
                                () ->
                                    RobotState.getInstance().getEstimatedPose().getY()
                                        < FieldConstants.StagingLocations.centerlineFirstY + 0.5),

                        // Finish intaking
                        waitUntilXCrossed(FieldConstants.wingX, false)
                            .deadlineWith(
                                rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE),
                                superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
                                flywheels.shootCommand()),

                        // Prepare shot
                        superstructure
                            .setGoalCommand(Superstructure.Goal.AIM)
                            .alongWith(flywheels.shootCommand()))),

            // Shoot centerline note
            rollers
                .setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER)
                .withTimeout(shootTimeoutSecs.get())
                .deadlineWith(
                    superstructure.setGoalCommand(Superstructure.Goal.AIM),
                    flywheels.shootCommand()),

            // Grab and shoot podium note
            followTrajectory(drive, grabPodium)
                .andThen(Commands.waitSeconds(0.5))
                .deadlineWith(
                    rollers.setGoalCommand(Rollers.Goal.QUICK_INTAKE_TO_FEED),
                    superstructure.setGoalCommand(Superstructure.Goal.AIM),
                    flywheels.shootCommand()));
  }
}
