// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;

public class AutoCommands {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;

  public AutoCommands(
      Drive drive, Superstructure superstructure, Flywheels flywheels, Rollers rollers) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.flywheels = flywheels;
    this.rollers = rollers;
  }

  private Command path(String pathName) {
    HolonomicTrajectory trajectory = new HolonomicTrajectory(pathName);

    return startEnd(
            () -> {
              drive.setTrajectoryGoal(trajectory);
            },
            () -> {
              drive.clearTrajectoryGoal();
            })
        .until(() -> drive.isTrajectoryGoalCompleted());
  }

  private Command reset(String path) {
    HolonomicTrajectory trajectory = new HolonomicTrajectory(path);
    return runOnce(
        () ->
            RobotState.getInstance().resetPose(AllianceFlipUtil.apply(trajectory.getStartPose())));
  }

  public Command davisEthicalAuto() {
    return sequence(
            // TODO: update to use the new commands
            reset("davisEthicalAuto_driveToPodium"),
            Commands.waitUntil(() -> superstructure.atGoal() && flywheels.atGoal())
                .andThen(rollers.feedShooter().withTimeout(.1))
                .deadlineWith(superstructure.aim()),
            path("davisEthicalAuto_driveToPodium")
                .deadlineWith(
                    superstructure
                        .intake()
                        .alongWith(rollers.floorIntake())
                        .withTimeout(3)
                        .andThen(superstructure.aim())),
            Commands.waitUntil(superstructure::atGoal),
            rollers.feedShooter().withTimeout(.1).deadlineWith(superstructure.aim()),
            path("davisEthicalAuto_driveToCenterline2")
                .deadlineWith(
                    superstructure
                        .intake()
                        .alongWith(rollers.floorIntake())
                        .withTimeout(3)
                        .andThen(superstructure.aim())),
            Commands.waitUntil(superstructure::atGoal),
            rollers.feedShooter().withTimeout(.1).deadlineWith(superstructure.aim()),
            path("davisEthicalAuto_driveToCenterline1")
                .deadlineWith(
                    superstructure
                        .intake()
                        .alongWith(rollers.floorIntake())
                        .withTimeout(3)
                        .andThen(superstructure.aim())),
            Commands.waitUntil(superstructure::atGoal),
            path("davisEthicalAuto_driveToCenterline0")
                .deadlineWith(
                    superstructure
                        .intake()
                        .alongWith(rollers.floorIntake())
                        .withTimeout(3)
                        .andThen(superstructure.aim()))

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
            )
        .deadlineWith(flywheels.shootCommand());
  }

  public Command N5_S1_C234() {
    return sequence(
        reset("N5-S1-C234_driveToS1"),
        path("N5-S1-C234_driveToS1"),
        path("N5-S1-C234_driveToC2"),
        path("N5-S1-C234_driveToC3"),
        path("N5-S1-C234_driveToC4"));
  }

  public Command N5_S0_C012() {
    return sequence(
        reset("N5-S0-C0123_driveToS0"),
        //        Commands.waitUntil(
        //                () -> true
        //                                () -> superstructure.atArmSetpoint() &&
        // flywheels.atSetpoint()
        //                )
        //            .andThen(rollers.feedShooterCommand().withTimeout(.3))
        //            .deadlineWith(superstructure.aimCommand()),
        path("N5-S0-C0123_driveToS0"),
        //            .deadlineWith(
        //                superstructure
        //                    .floorIntakeCommand()
        //                    .alongWith(rollers.floorIntakeCommand())
        //                    .withTimeout(3)
        //                    .andThen(superstructure.aimCommand())),
        //
        // rollers.feedShooterCommand().withTimeout(.1).deadlineWith(superstructure.aimCommand()),
        path("N5-S0-C0123_driveToC0")
        //            .deadlineWith(
        //                superstructure
        //                    .floorIntakeCommand()
        //                    .alongWith(rollers.floorIntakeCommand())
        //                    .withTimeout(3)
        //                    .andThen(superstructure.aimCommand())),
        //
        // rollers.feedShooterCommand().withTimeout(.1).deadlineWith(superstructure.aimCommand())
        //        path("N5-S0-C0123_driveToC1")
        //            .deadlineWith(
        //                superstructure
        //                    .floorIntakeCommand()
        //                    .alongWith(rollers.floorIntakeCommand())
        //                    .withTimeout(3)
        //                    .andThen(superstructure.aimCommand())),
        //
        // rollers.feedShooterCommand().withTimeout(.1).deadlineWith(superstructure.aimCommand())
        //            path("N5-S0-C0123_driveToC2")
        //                .deadlineWith(superstructure.floorIntakeCommand(),
        // rollers.floorIntakeCommand()),
        //            Commands.waitUntil(
        //                    () -> true
        //                    //                superstructure::atArmSetpoint
        //                    )
        //                .andThen(rollers.feedShooterCommand().withTimeout(.1)))
        //        .deadlineWith(flywheels.shootCommand()
        );
  }

  public Command driveStraight() {
    return reset("driveToCenterline4")
        .andThen(path("driveToCenterline4"), path("driveToCenterline3"), path("driveToPodium"));
  }
}
