// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands;

import static org.littletonrobotics.frc2024.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@ExtensionMethod({GeomUtil.class})
public class ClimbingCommands {
  private static final LoggedTunableNumber climbedXOffset =
      new LoggedTunableNumber("ClimbingCommands/ClimbedXOffset", 0.16);
  private static final LoggedTunableNumber chainToBack =
      new LoggedTunableNumber("ClimbingCommands/ChainToBackOffset", -0.5);
  private static final LoggedTunableNumber edgeChainMargin =
          new LoggedTunableNumber("ClimbingCommands/EdgeChainMargin", 0.1);

  private static final List<Pose2d> centeredClimbedPosesNoOffset =
      List.of(
          Stage.centerPodiumAmpChain, Stage.centerAmpSourceChain, Stage.centerSourcePodiumChain);

  private static final List<List<Pose2d>> stageLegPosesNoOffset =
      List.of(
          List.of(Stage.podiumLeg, Stage.ampLeg), // Podium to Amp
          List.of(Stage.ampLeg, Stage.sourceLeg), // Amp to source side
          List.of(Stage.sourceLeg, Stage.podiumLeg));

  static {
    // Set rotations of edge poses
    for (int i = 0; i < 3; i++) {
      Rotation2d climbedHeading = centeredClimbedPosesNoOffset.get(i).getRotation();
      stageLegPosesNoOffset.set(
          i,
          stageLegPosesNoOffset.get(i).stream()
              .map(pose -> new Pose2d(pose.getTranslation(), climbedHeading))
              .toList());
    }
  }

  private static final Supplier<Pose2d> nearestClimbedPose =
      () -> {
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        List<Pose2d> climbedPoses =
            centeredClimbedPosesNoOffset.stream()
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
              .transformBy(new Translation2d(chainToBack.get(), 0).toTransform2d());

  /** Drive to back climber ready pose. */
  public static Command driveToBack(Drive drive) {
    return drive
        .startEnd(
            () -> drive.setAutoAlignGoal(backedPrepareClimberPose, false),
            drive::clearAutoAlignGoal)
        .until(drive::isAutoAlignGoalCompleted);
  }

  /**
   * Drives to climbed pose while raising climber up and arm back, ends when at position with drive
   * and superstructure.
   */
  private static Command prepareClimbFromBack(
      Drive drive, Superstructure superstructure, Trigger autoDriveDisable) {
    return drive.startEnd(
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              Pose2d targetPose =
                  currentPose.transformBy(new Translation2d(-chainToBack.get(), 0).toTransform2d());
              drive.setAutoAlignGoal(() -> targetPose, true);
            },
            drive::clearAutoAlignGoal)
        .onlyIf(autoDriveDisable.negate())
        .alongWith(superstructure.setGoalCommand(Superstructure.Goal.PREPARE_CLIMB));
  }

  /** Runs the climbing sequence and then scores in the trap when the trapScore button is held */
  public static Command climbSequence(
      Drive drive,
      Superstructure superstructure,
      Rollers rollers,
      Trigger startClimbTrigger,
      Trigger trapScoreTrigger,
      Trigger autoDriveDisable) {
    return Commands.sequence(
            // Drive forward while raising arm and climber
            prepareClimbFromBack(drive, superstructure, autoDriveDisable)
                .until(() -> superstructure.atGoal()),

            // Allow driver to line up
            superstructure
                .setGoalCommand(Superstructure.Goal.PREPARE_CLIMB)
                .until(() -> startClimbTrigger.getAsBoolean()),

            // Climb and wait, continue if trap button pressed
            superstructure
                .setGoalCommand(Superstructure.Goal.CLIMB)
                .until(() -> trapScoreTrigger.getAsBoolean() && superstructure.atGoal()),

            // Shuffle to backpack
            rollers
                .setGoalCommand(Rollers.Goal.SHUFFLE_BACKPACK)
                .alongWith(superstructure.setGoalCommand(Superstructure.Goal.CLIMB))
                .until(() -> rollers.getGamepieceState() == Rollers.GamepieceState.BACKPACK_STAGED),

            // Extend backpack
            superstructure.setGoalCommand(Superstructure.Goal.TRAP).until(superstructure::atGoal),

            // Score in trap and wait
            rollers
                .setGoalCommand(Rollers.Goal.TRAP_SCORE)
                .alongWith(superstructure.setGoalCommand(Superstructure.Goal.TRAP)))

        // If cancelled, go to safe state
        .finallyDo(
            () -> {
              switch (superstructure.getCurrentGoal()) {
                case PREPARE_CLIMB ->
                    superstructure.setDefaultCommand(
                        superstructure.setGoalCommand(Superstructure.Goal.CANCEL_PREPARE_CLIMB));
                case CLIMB ->
                    superstructure.setDefaultCommand(
                        superstructure.setGoalCommand(Superstructure.Goal.CANCEL_CLIMB));
                case TRAP ->
                    superstructure.setDefaultCommand(
                        superstructure.setGoalCommand(Superstructure.Goal.CLIMB));
              }
            });
  }
}
