package org.littletonrobotics.frc2024.commands.auto;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.trajectory.HolonomicTrajectory;

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

  public Command driveStraight() {
    return null; // reset("driveStraight").andThen(path("driveStraight"));
  }
}
