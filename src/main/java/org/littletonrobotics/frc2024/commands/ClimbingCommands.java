// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@ExtensionMethod({GeomUtil.class})
public class ClimbingCommands {
  private static final LoggedTunableNumber climbedXOffset =
      new LoggedTunableNumber(
          "ClimbingCommands/ClimbedXOffset",
          -ArmConstants.armOrigin.getX() + Units.inchesToMeters(2.0));
  private static final LoggedTunableNumber chainToBack =
      new LoggedTunableNumber("ClimbingCommands/ChainToBackOffset", 0.5);

  private static final List<Pose2d> climbedPosesNoOffset =
      List.of(
          FieldConstants.Stage.centerPodiumAmpChain,
          FieldConstants.Stage.centerAmpSourceChain,
          FieldConstants.Stage.centerSourcePodiumChain);

  private static final Supplier<Pose2d> nearestClimbedPose =
      () -> {
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        List<Pose2d> climbedPoses =
            climbedPosesNoOffset.stream()
                .map(
                    pose ->
                        AllianceFlipUtil.apply(
                            pose.transformBy(
                                new Translation2d(climbedXOffset.get(), 0).toTransform2d())))
                .toList();
        return currentPose.nearest(climbedPoses);
      };
  private static final Supplier<Pose2d> backedPrepareClimberPose =
      () ->
          nearestClimbedPose
              .get()
              .transformBy(new Translation2d(-chainToBack.get(), 0).toTransform2d());

  /** Drive to back climber ready pose. */
  public static Command driveToBack(Drive drive) {
    return Commands.startEnd(
        () -> drive.setAutoAlignGoal(backedPrepareClimberPose, false), drive::clearAutoAlignGoal);
  }

  /**
   * Drives to climbed pose while raising climber up and arm back, ends when at position with drive
   * and superstructure.
   */
  public static Command prepareClimbFromBack(Drive drive, Superstructure superstructure) {
    return Commands.startEnd(
            () -> drive.setAutoAlignGoal(nearestClimbedPose, true), drive::clearAutoAlignGoal)
        .alongWith(superstructure.setGoalCommand(Superstructure.Goal.PREPARE_CLIMB));
  }

  /** Pulls down with climber while arm moves to climb position (90), never ends. */
  public static Command finalClimb(Superstructure superstructure) {
    return superstructure.setGoalCommand(Superstructure.Goal.CLIMB);
  }
}
