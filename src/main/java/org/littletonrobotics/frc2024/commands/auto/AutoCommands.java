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
  private final Drive drive;
  private final Superstructure superstructure;

  public AutoCommands(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
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
        reset("driveToCenterline4"),
        path("driveToCenterline4"),
        path("driveToCenterline3"),
        path("driveToPodium"));
  }
  ;

  public Command driveStraight() {
    return reset("driveToCenterline4")
        .andThen(path("driveToCenterline4"), path("driveToCenterline3"), path("driveToPodium"));
  }
  ;
}
