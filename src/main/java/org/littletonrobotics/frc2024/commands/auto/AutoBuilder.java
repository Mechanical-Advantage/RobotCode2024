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
import java.util.List;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.DriveTrajectories;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;

@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Flywheels flywheels;
  private final Rollers rollers;
  private final Supplier<List<AutoQuestionResponse>> responses;

  public Command davisSpikyAuto() {
    return Commands.none();
  }

  public Command davisSpeedyAuto() {
    var grabCenterline4 = new HolonomicTrajectory("speedy_ampToCenterline4");
    var grabCenterline3 = new HolonomicTrajectory("speedy_centerline4ToCenterline3");
    var grabCenterline2 = new HolonomicTrajectory("speedy_centerline3ToCenterline2");
    var grabEjected = new HolonomicTrajectory("speedy_centerline2ToEjectedNote");

    Timer autoTimer = new Timer();
    return Commands.runOnce(autoTimer::restart)
        .andThen(
            Commands.sequence(
                resetPose(DriveTrajectories.startingAmpWall),
                followTrajectory(drive, grabCenterline4),
                followTrajectory(drive, grabCenterline3),
                followTrajectory(drive, grabCenterline2),
                followTrajectory(drive, grabEjected)));
  }

  public Command davisEthicalAuto() {
    return Commands.none();
  }

  public Command davisUnethicalAuto() {
    return Commands.none();
  }
}
