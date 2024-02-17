// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.littletonrobotics.frc2024.commands.auto.AutoCommands.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
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
        // Shoot first note
        resetPose(driveToPodiumTrajectory),
        shoot(superstructure, flywheels, rollers),

        // Drive to podium while intaking then shoot
        followTrajectory(drive, driveToPodiumTrajectory)
            .deadlineWith(intake(superstructure, rollers).andThen(superstructure.aim())),
        shoot(superstructure, flywheels, rollers),

        // Drive to centerline waiting to intake after we cross the wing
        followTrajectory(drive, driveToCenterline2Trajectory)
            .deadlineWith(
                waitUntilXCrossed(FieldConstants.wingX).andThen(intake(superstructure, rollers))),
        shoot(superstructure, flywheels, rollers),

        // Drive back to centerline 1
        followTrajectory(drive, driveToCenterline1Trajectory)
            .deadlineWith(intake(superstructure, rollers)),
        shoot(superstructure, flywheels, rollers),

        // Drive to centerline 0 and come back for shot
        followTrajectory(drive, driveToCenterline0Trajectory)
            .deadlineWith(intake(superstructure, rollers)),
        shoot(superstructure, flywheels, rollers)

        //

        //            path("davisEthicalAuto_driveToCenterline2")
        //                .deadlineWith(
        //                    superstructure
        //                        .floorIntakeCommand()
        //                        .alongWith(rollers.floorIntakeCommand())
        //                        .withTimeout(3)
        //                        .andThen(superstructure.aimCommand())),
        //
        // rollers.feedShooterCommand().withTimeout(.1).deadlineWith(superstructure.aimCommand()),
        //            path("davisEthicalAuto_driveToPodium")
        //                .deadlineWith(superstructure.floorIntakeCommand(),
        // rollers.floorIntakeCommand()),
        //            Commands.waitUntil(
        //                    () -> true
        //                    //                superstructure::atArmSetpoint
        //                    )
        //                .andThen(rollers.feedShooterCommand().withTimeout(.1))
        );
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
