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
    HolonomicTrajectory driveToPodiumTrajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToPodium");
    HolonomicTrajectory driveToCenterline2Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline2");
    HolonomicTrajectory driveToCenterline1Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline1");
    HolonomicTrajectory driveToCenterline0Trajectory =
        new HolonomicTrajectory("davisEthicalAuto_driveToCenterline0");

    return sequence(
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.AUTO)),
        // Shoot preloaded note
        resetPose(driveToPodiumTrajectory),
        shoot(drive, superstructure, flywheels, rollers),
        print("First shot at " + Timer.getFPGATimestamp()),

        // Drive to podium note while intaking and shoot
        followTrajectory(drive, driveToPodiumTrajectory)
            .deadlineWith(
                intake(superstructure, rollers)), // TODO: change back to alongWith for real
        shoot(drive, superstructure, flywheels, rollers),
        print("Second shot at " + Timer.getFPGATimestamp()),

        // Drive to centerline 2 note making sure to only intake after crossed stage
        followTrajectory(drive, driveToCenterline2Trajectory)
            .deadlineWith(
                sequence(
                    // Check if full length of robot + some has passed wing for arm safety
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(0.8),
                    // Wait until we are close enough to shot to start arm aiming
                    waitUntilXCrossed(FieldConstants.Stage.podiumLeg.getX() + 0.2, false),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        print("Third shot at " + Timer.getFPGATimestamp()),

        // Drive back to centerline 1 while intaking
        followTrajectory(drive, driveToCenterline1Trajectory)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(1.0),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        print("Fourth shot at " + Timer.getFPGATimestamp()),

        // Drive back to centerline 0 and then shoot
        followTrajectory(drive, driveToCenterline0Trajectory)
            .deadlineWith(
                sequence(
                    waitUntilXCrossed(
                        FieldConstants.wingX + DriveConstants.driveConfig.bumperWidthX() * 0.7,
                        true),
                    intake(superstructure, rollers).withTimeout(1.0),
                    superstructure.aim())),
        shoot(drive, superstructure, flywheels, rollers),
        print("Fifth shot at " + Timer.getFPGATimestamp()),
        // Revert to teleop idle mode
        runOnce(() -> flywheels.setIdleMode(Flywheels.IdleMode.TELEOP)));
  }

  //  public Command N5_S1_C234() {
  //    return sequence(
  //        reset("N5-S1-C234_driveToS1"),
  //        path("N5-S1-C234_driveToS1"),
  //        path("N5-S1-C234_driveToC2"),
  //        path("N5-S1-C234_driveToC3"),
  //        path("N5-S1-C234_driveToC4"));
  //  }

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
