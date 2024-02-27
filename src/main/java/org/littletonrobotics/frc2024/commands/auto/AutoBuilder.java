// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
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

    return Commands.sequence(
        resetAndFollow(drive, driveToPodium),
        Commands.waitSeconds(0.3),
        followTrajectory(drive, grabCenterline0),
        followTrajectory(drive, grabCenterline1),
        followTrajectory(drive, grabCenterline2));

    // HolonomicTrajectory driveToPodiumTrajectory =
    //     new HolonomicTrajectory("davisEthicalAuto_driveToPodium");
    // HolonomicTrajectory driveToCenterline2Trajectory =
    //     new HolonomicTrajectory("davisEthicalAuto_driveToCenterline2");
    // HolonomicTrajectory driveToCenterline1Trajectory =
    //     new HolonomicTrajectory("davisEthicalAuto_driveToCenterline1");
    // HolonomicTrajectory driveToCenterline0Trajectory =
    //     new HolonomicTrajectory("davisEthicalAuto_driveToCenterline0");

    // Timer autoTimer = new Timer();
    // return sequence(
    //     runOnce(autoTimer::restart),
    //     // Shoot preloaded note
    //     resetPose(driveToPodiumTrajectory),
    //     shootNoDrive(superstructure, flywheels, rollers),
    //     runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
    //     followTrajectory(drive, driveToPodiumTrajectory)
    //         // Drive to podium note while intaking and shoot
    //         .deadlineWith(
    //             parallel(intake(superstructure, rollers), flywheels.shootCommand())), // uh oh 👀
    //     shoot(drive, superstructure, flywheels, rollers),
    //     runOnce(() -> System.out.printf("First shot at %.2f seconds.", autoTimer.get())),
    //     // NoteVisualizer.shoot(),
    //     runOnce(() -> System.out.printf("Second shot at %.2f seconds.", autoTimer.get())),

    //     // Drive to centerline 2 note making sure to only intake after crossed stage
    //     followTrajectory(drive, driveToCenterline2Trajectory)
    //         .deadlineWith(
    //             sequence(
    //                 // Check if full length of robot + some has passed wing for arm safety
    //                 waitUntilXCrossed(
    //                     FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(), true),
    //                 intake(superstructure, rollers)
    //                     .raceWith(
    //                         waitUntilXCrossed(
    //                             FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(),
    //                             false)),
    //                 // Wait until we are close enough to shot to start arm aiming
    //                 waitUntilXCrossed(FieldConstants.Stage.podiumLeg.getX() + 0.5, false),
    //                 parallel(intake(superstructure, rollers), flywheels.shootCommand()))),
    //     shoot(drive, superstructure, flywheels, rollers),
    //     runOnce(() -> System.out.printf("Third shot at %.2f seconds.", autoTimer.get())),

    //     // Drive back to centerline 1 while intaking
    //     followTrajectory(drive, driveToCenterline1Trajectory)
    //         .deadlineWith(
    //             sequence(
    //                 waitUntilXCrossed(
    //                     FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX(), true),
    //                 intake(superstructure, rollers)
    //                     .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
    //                 parallel(intake(superstructure, rollers), flywheels.shootCommand()))),
    //     shoot(drive, superstructure, flywheels, rollers),
    //     runOnce(() -> System.out.printf("Fourth shot at %.2f seconds.", autoTimer.get())),

    //     // Drive back to centerline 0 and then shoot
    //     followTrajectory(drive, driveToCenterline0Trajectory)
    //         .deadlineWith(
    //             sequence(
    //                 waitUntilXCrossed(
    //                     FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
    //                     true),
    //                 intake(superstructure, rollers)
    //                     .raceWith(waitUntilXCrossed(FieldConstants.wingX, false)),
    //                 parallel(intake(superstructure, rollers), flywheels.shootCommand()))),
    //     shoot(drive, superstructure, flywheels, rollers),
    //     runOnce(() -> System.out.printf("Fifth shot at %.2f seconds.", autoTimer.get())));
  }
}
