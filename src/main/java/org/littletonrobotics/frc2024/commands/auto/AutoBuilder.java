// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.frc2024.FieldConstants;
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

  public Command N4_S0_C21() {
    Timer autoTimer = new Timer();
    HolonomicTrajectory driveToPodiumTrajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToPodium");
    HolonomicTrajectory driveToCenterline2Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline2");
    HolonomicTrajectory driveToCenterline1Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline1");

    return sequence(
        runOnce(autoTimer::restart),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        // Shoot preloaded note
        resetPose(driveToPodiumTrajectory),
        shootNoDrive(superstructure, flywheels, rollers),
        followTrajectory(drive, driveToPodiumTrajectory)
            // Drive to podium note while intaking and shoot
            .deadlineWith(intake(superstructure, rollers)),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
        // NoteVisualizer.shoot(),
        runOnce(() -> System.out.printf("Second shot at %.2f seconds.", autoTimer.get())),

        // Drive to centerline 2 note making sure to only intake after crossed stage
        followTrajectory(drive, driveToCenterline2Trajectory)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(), true),
                    intake(superstructure, rollers)
                        .raceWith(
                            waitUntilXCrossed(
                                FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(),
                                false)),
                    // Wait until we are close enough to shot to start arm aiming
                    waitUntilXCrossed(FieldConstants.Stage.podiumLeg.getX() + 0.5, false),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Third shot at %.2f seconds.", autoTimer.get())),

        // Drive back to centerline 1 while intaking
        followTrajectory(drive, driveToCenterline1Trajectory)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(), true),
                    intake(superstructure, rollers)
                        .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  public Command N4_S2_C43() {
    Timer autoTimer = new Timer();
    HolonomicTrajectory driveToS2 = new HolonomicTrajectory("N5-S2-C432_driveToS2");
    HolonomicTrajectory driveToC4 = new HolonomicTrajectory("N5-S2-C432_driveToC4");
    HolonomicTrajectory driveToC3 = new HolonomicTrajectory("N5-S2-C432_driveToC3");

    return sequence(
        runOnce(autoTimer::restart),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        resetPose(driveToS2),
        shootNoDrive(superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("First shot at %.2f seconds. ", autoTimer.get())),
        followTrajectory(drive, driveToS2).deadlineWith(intake(superstructure, rollers)),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Second shot at %.2f seconds. ", autoTimer.get())),
        followTrajectory(drive, driveToC4)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers)
                        .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Third shot at %.2f seconds. ", autoTimer.get())),
        followTrajectory(drive, driveToC3)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers)
                        .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds. ", autoTimer.get())),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  public Command davisEthicalAuto() {
    HolonomicTrajectory driveToPodiumTrajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToPodium");
    HolonomicTrajectory driveToCenterline2Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline2");
    HolonomicTrajectory driveToCenterline1Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline1");
    HolonomicTrajectory driveToCenterline0Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline0");

    Timer autoTimer = new Timer();
    return sequence(
        runOnce(autoTimer::restart),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        // Shoot preloaded note
        resetPose(driveToPodiumTrajectory),
        shootNoDrive(superstructure, flywheels, rollers),
        followTrajectory(drive, driveToPodiumTrajectory)
            // Drive to podium note while intaking and shoot
            .deadlineWith(intake(superstructure, rollers)), // uh oh 👀
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
        // NoteVisualizer.shoot(),
        runOnce(() -> System.out.printf("Second shot at %.2f seconds.", autoTimer.get())),

        // Drive to centerline 2 note making sure to only intake after crossed stage
        followTrajectory(drive, driveToCenterline2Trajectory)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(), true),
                    intake(superstructure, rollers)
                        .raceWith(
                            waitUntilXCrossed(
                                FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(),
                                false)),
                    // Wait until we are close enough to shot to start arm aiming
                    waitUntilXCrossed(FieldConstants.Stage.podiumLeg.getX() + 0.5, false),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Third shot at %.2f seconds.", autoTimer.get())),

        // Drive back to centerline 1 while intaking
        followTrajectory(drive, driveToCenterline1Trajectory)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(), true),
                    intake(superstructure, rollers)
                        .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),

        // Drive back to centerline 0 and then shoot
        followTrajectory(drive, driveToCenterline0Trajectory)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers)
                        .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fifth shot at %.2f seconds.", autoTimer.get())),
        // Revert to teleop idle mode
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  public Command N5_C012_S0() {
    HolonomicTrajectory driveToC0 = new HolonomicTrajectory("N5-C012-S0_driveToC0");
    HolonomicTrajectory driveToC1 = new HolonomicTrajectory("N5-C012-S0_driveToC1");
    HolonomicTrajectory driveToC2 = new HolonomicTrajectory("N5-C012-S0_driveToC2");
    HolonomicTrajectory driveToPodium = new HolonomicTrajectory("N5-C012-S0_driveToPodium");
    Timer autoTimer = new Timer();

    return sequence(
        runOnce(autoTimer::restart),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        resetPose(driveToC0),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC0)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Second shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC1)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Third shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC2)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    waitUntilXCrossed(FieldConstants.Stage.podiumLeg.getX() + 0.5, false),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToPodium).deadlineWith(intake(superstructure, rollers)),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fifth shot at %.2f seconds.", autoTimer.get())),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  public Command N5_S1_C234() {
    Timer autoTimer = new Timer();
    HolonomicTrajectory driveToS1 = new HolonomicTrajectory("N5-S1-C234_driveToS1");
    HolonomicTrajectory driveToC2 = new HolonomicTrajectory("N5-S1-C234_driveToC2");
    HolonomicTrajectory driveToC3 = new HolonomicTrajectory("N5-S1-C234_driveToC3");
    HolonomicTrajectory driveToC4 = new HolonomicTrajectory("N5-S1-C234_driveToC4");

    return sequence(
        runOnce(autoTimer::restart),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        // Shoot preloaded note
        resetPose(driveToS1),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToS1).deadlineWith(intake(superstructure, rollers)),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Second shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC2)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    // Wait until we are close enough to shot to start arm aiming
                    waitUntilXCrossed(FieldConstants.Stage.podiumLeg.getX() + 0.5, false),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Third shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC3)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(1.0),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC4)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(1.0),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fifth shot at %.2f seconds.", autoTimer.get())),
        // Revert to teleop idle mode
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  public Command N5_S2_C432() {
    Timer autoTimer = new Timer();
    HolonomicTrajectory driveToS2 = new HolonomicTrajectory("N5-S2-C432_driveToS2");
    HolonomicTrajectory driveToC4 = new HolonomicTrajectory("N5-S2-C432_driveToC4");
    HolonomicTrajectory driveToC3 = new HolonomicTrajectory("N5-S2-C432_driveToC3");
    HolonomicTrajectory driveToC2 = new HolonomicTrajectory("N5-S2-C432_driveToC2");

    return sequence(
        runOnce(autoTimer::restart),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        resetPose(driveToS2),
        shootNoDrive(superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToS2).deadlineWith(intake(superstructure, rollers)),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Second shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC4)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers)
                        .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Third shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC3)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers)
                        .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC2)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(), true),
                    intake(superstructure, rollers)
                        .raceWith(
                            waitUntilXCrossed(
                                FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(),
                                false)),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  public Command N6_S21_C432() {
    Timer autoTimer = new Timer();
    HolonomicTrajectory driveToS2 = new HolonomicTrajectory("N6-S21-C432_driveToS2");
    HolonomicTrajectory driveToS1 = new HolonomicTrajectory("N6-S21-C432_driveToS1");
    HolonomicTrajectory driveToC4 = new HolonomicTrajectory("N6-S21-C432_driveToC4");
    HolonomicTrajectory driveToC3 = new HolonomicTrajectory("N6-S21-C432_driveToC3");
    HolonomicTrajectory driveToC2 = new HolonomicTrajectory("N6-S21-C432_driveToC2");
    return sequence(
        runOnce(autoTimer::restart),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        resetPose(driveToS2),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToS2).deadlineWith(intake(superstructure, rollers)),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Second shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToS1).deadlineWith(intake(superstructure, rollers)),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Third shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC4)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC3)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Fifth shot at %.2f seconds.", autoTimer.get())),
        followTrajectory(drive, driveToC2)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        runOnce(() -> System.out.printf("Sixth shot at %.2f seconds.", autoTimer.get())),
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  //  public Command N5_S0_C012() {
  //    return sequence(
  //        reset("N5-S0-C0123_driveToS0"),
  //        //        Commands.waitUntil(
  //        //                () -> true
  //        //                                () -> superstructure.atArmSetpoint() &&
  //        // flywheels.atSetpoint()
  //        //                )
  //        //            .andThen(rollers.feedShooterCommand().withTimeout(.3))
  //        //            .deadlineWith(superstructure.aimCommand()),
  //        path("N5-S0-C0123_driveToS0"),
  //        //            .deadlineWith(
  //        //                superstructure
  //        //                    .floorIntakeCommand()
  //        //                    .alongWith(rollers.floorIntakeCommand())
  //        //                    .withTimeout(3)
  //        //                    .andThen(superstructure.aimCommand())),
  //        //
  //        //
  // rollers.feedShooterCommand().withTimeout(.1).deadlineWith(superstructure.aimCommand()),
  //        path("N5-S0-C0123_driveToC0")
  //        //            .deadlineWith(
  //        //                superstructure
  //        //                    .floorIntakeCommand()
  //        //                    .alongWith(rollers.floorIntakeCommand())
  //        //                    .withTimeout(3)
  //        //                    .andThen(superstructure.aimCommand())),
  //        //
  //        //
  // rollers.feedShooterCommand().withTimeout(.1).deadlineWith(superstructure.aimCommand())
  //        //        path("N5-S0-C0123_driveToC1")
  //        //            .deadlineWith(
  //        //                superstructure
  //        //                    .floorIntakeCommand()
  //        //                    .alongWith(rollers.floorIntakeCommand())
  //        //                    .withTimeout(3)
  //        //                    .andThen(superstructure.aimCommand())),
  //        //
  //        //
  // rollers.feedShooterCommand().withTimeout(.1).deadlineWith(superstructure.aimCommand())
  //        //            path("N5-S0-C0123_driveToC2")
  //        //                .deadlineWith(superstructure.floorIntakeCommand(),
  //        // rollers.floorIntakeCommand()),
  //        //            Commands.waitUntil(
  //        //                    () -> true
  //        //                    //                superstructure::atArmSetpoint
  //        //                    )
  //        //                .andThen(rollers.feedShooterCommand().withTimeout(.1)))
  //        //        .deadlineWith(flywheels.shootCommand()
  //        );
}

//  public Command driveStraight() {
//    return reset("driveToPodium")
//        .andThen(path("driveToPodium"));
