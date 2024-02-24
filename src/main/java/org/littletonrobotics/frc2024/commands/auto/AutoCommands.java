// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;

public class AutoCommands {
  private Drive drive;
  private Superstructure superstructure;

  public AutoCommands(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
  }

  private Command path(String pathName) {
    HolonomicTrajectory trajectory = new HolonomicTrajectory(pathName);

    return startEnd(
            () -> {
              drive.setTrajectory(trajectory);
            },
            () -> {
              drive.clearTrajectory();
            })
        .until(() -> drive.isTrajectoryCompleted());
  }

  private Command reset(String path) {
    HolonomicTrajectory trajectory = new HolonomicTrajectory(path);
    return runOnce(
        () ->
            RobotState.getInstance().resetPose(AllianceFlipUtil.apply(trajectory.getStartPose())));
  }

  public Command driveStraight() {
    return reset("driveStraight").andThen(path("driveStraight"));
  }

  public Command driveStraightShooting() {
    return reset("driveStraight")
        .andThen(
            startEnd(
                () -> drive.setShootingTrajectory(new HolonomicTrajectory("driveStraight")),
                drive::clearShootingTrajectory));
  }
}
