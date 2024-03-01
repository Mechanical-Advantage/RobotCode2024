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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@ExtensionMethod({GeomUtil.class})
public class ClimbingCommands {
  private static final LoggedTunableNumber climbedXOffset =
      new LoggedTunableNumber("ClimbingCommands/ClimbedXOffset", 0.16);
  private static final LoggedTunableNumber chainToBack =
      new LoggedTunableNumber("ClimbingCommands/ChainToBackOffset", 0.5);
  private static final LoggedTunableNumber chainToFront =
      new LoggedTunableNumber("ClimbingCommands/ChainToFrontOffset", 0.9);

  private static final List<Pose2d> centeredClimbedPosesNoOffset =
      List.of(
          Stage.centerPodiumAmpChain, Stage.centerAmpSourceChain, Stage.centerSourcePodiumChain);

  private static final Supplier<Pose2d> nearestClimbedPose =
      () -> {
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        List<Pose2d> climbedPoses =
            centeredClimbedPosesNoOffset.stream()
                .map(pose -> AllianceFlipUtil.apply(pose))
                .toList();
        return currentPose.nearest(climbedPoses);
      };
  private static final Supplier<Pose2d> backPrepareClimbPose =
      () ->
          nearestClimbedPose
              .get()
              .transformBy(
                  new Translation2d(climbedXOffset.get() - chainToBack.get(), 0).toTransform2d());
  private static final Supplier<Pose2d> frontPrepareClimbPose =
      () ->
          nearestClimbedPose
              .get()
              .transformBy(
                  new Translation2d(-climbedXOffset.get() + chainToFront.get(), 0).toTransform2d())
              .transformBy(
                  new Transform2d(
                      0, 0, Rotation2d.fromDegrees(180.0))); // Flip climbed pose for front climb

  /** Command that lets driver adjust robot relative to the robot at slow speed */
  private static Command driverAdjust(
      Drive drive, DoubleSupplier controllerX, DoubleSupplier controllerY) {
    return drive.run(
        () ->
            drive.acceptTeleopInput(
                controllerX.getAsDouble() * 0.25, controllerY.getAsDouble() * 0.25, 0, true));
  }

  private static Command driveToPoseWithAdjust(
      Drive drive,
      Supplier<Pose2d> prepareClimbPose,
      DoubleSupplier controllerX,
      DoubleSupplier controllerY) {
    return Commands.sequence(
        // Auto drive to behind the chain
        drive
            .startEnd(
                () -> drive.setAutoAlignGoal(prepareClimbPose, false), drive::clearAutoAlignGoal)
            .until(drive::isAutoAlignGoalCompleted),

        // Let driver move robot left and right while aligned to chain
        driverAdjust(drive, controllerX, controllerY));
  }

  /** Drive to back climber ready pose. */
  public static Command autoDrive(
      boolean isFront,
      Drive drive,
      DoubleSupplier controllerX,
      DoubleSupplier controllerY,
      Trigger autoDriveDisable) {
    return driveToPoseWithAdjust(
            drive, isFront ? frontPrepareClimbPose : backPrepareClimbPose, controllerX, controllerY)
        .onlyIf(autoDriveDisable.negate());
  }

  /**
   * Drives to climbed pose while raising climber up and arm back, ends when at position with drive
   * and superstructure.
   */
  private static Command prepareClimbFromBack(
      Drive drive, Superstructure superstructure, Trigger autoDriveDisable) {
    return drive
        .startEnd(
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              Pose2d targetPose =
                  currentPose.transformBy(new Translation2d(chainToBack.get(), 0).toTransform2d());
              drive.setAutoAlignGoal(() -> targetPose, true);
            },
            drive::clearAutoAlignGoal)
        .onlyIf(autoDriveDisable.negate())
        .alongWith(
            superstructure.setGoalWithConstraintsCommand(
                Superstructure.Goal.PREPARE_CLIMB, Arm.prepareClimbProfileConstraints.get()));
  }

  private static Command prepareClimbFromFront(
      Drive drive, Superstructure superstructure, Trigger autoDriveDisable) {
    return drive
        .startEnd(
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              Pose2d targetPose =
                  currentPose.transformBy(new Translation2d(chainToFront.get(), 0).toTransform2d());
              drive.setAutoAlignGoal(() -> targetPose, true);
            },
            drive::clearAutoAlignGoal)
        .onlyIf(autoDriveDisable.negate())
        .alongWith(superstructure.setGoalCommand(Superstructure.Goal.PREPARE_CLIMB));
  }

  public static Command simpleClimbSequence(
      Drive drive,
      Superstructure superstructure,
      DoubleSupplier controllerX,
      DoubleSupplier controllerY,
      Trigger startClimbTrigger,
      Trigger autoDriveDisable) {
    return Commands.sequence(
            // Drive forward
            prepareClimbFromFront(drive, superstructure, autoDriveDisable)
                .until(() -> superstructure.atGoal() && drive.isAutoAlignGoalCompleted())
                .withTimeout(5.0),

            // Allow driver to line up
            Commands.waitUntil(startClimbTrigger)
                .deadlineWith(
                    superstructure.setGoalCommand(Superstructure.Goal.PREPARE_CLIMB),
                    driverAdjust(drive, controllerX, controllerY)),

            // Climb
            superstructure.setGoalCommand(Superstructure.Goal.CLIMB))

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
              }
            });
  }

  /** Runs the climbing sequence and then scores in the trap when the trapScore button is pressed */
  public static Command climbNTrapSequence(
      Drive drive,
      Superstructure superstructure,
      Rollers rollers,
      DoubleSupplier controllerX,
      DoubleSupplier controllerY,
      Trigger startClimbTrigger,
      Trigger trapScoreTrigger,
      Trigger autoDriveDisable) {
    return Commands.sequence(
            // Drive forward while raising arm and climber
            prepareClimbFromBack(drive, superstructure, autoDriveDisable)
                .until(() -> superstructure.atGoal() && drive.isAutoAlignGoalCompleted())
                .withTimeout(5.0),

            // Allow driver to line up
            Commands.waitUntil(startClimbTrigger)
                .deadlineWith(
                    superstructure.setGoalCommand(Superstructure.Goal.PREPARE_CLIMB),
                    driverAdjust(drive, controllerX, controllerY)),

            // Climb and wait, continue if trap button pressed
            superstructure
                .setGoalCommand(Superstructure.Goal.CLIMB)
                .alongWith(rollers.setGoalCommand(Rollers.Goal.SHUFFLE_BACKPACK))
                .until(
                    () ->
                        trapScoreTrigger.getAsBoolean()
                            && superstructure.atGoal()
                            && rollers.getGamepieceState()
                                == Rollers.GamepieceState.BACKPACK_STAGED),

            // Extend backpack
            superstructure
                .setGoalCommand(Superstructure.Goal.TRAP)
                .alongWith(rollers.setGoalCommand(Rollers.Goal.TRAP_PRESCORE))
                .until(superstructure::atGoal),

            // Score in trap and wait
            rollers
                .setGoalCommand(Rollers.Goal.TRAP_SCORE)
                .alongWith(superstructure.setGoalCommand(Superstructure.Goal.TRAP))
                .until(() -> trapScoreTrigger.getAsBoolean()),

            // Retract backpack
            superstructure.setGoalCommand(Superstructure.Goal.CLIMB))

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
