// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.leds.Leds;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.frc2024.util.SteppableCommandGroup;

@ExtensionMethod({GeomUtil.class})
public class ClimbingCommands {
  private static final LoggedTunableNumber chainToBack =
      new LoggedTunableNumber("ClimbingCommands/ChainToBackOffset", 0.3);

  public static Command trapSequence(
      Drive drive,
      Superstructure superstructure,
      Rollers rollers,
      Trigger forwwardTrigger,
      Trigger reverseTrigger) {
    SteppableCommandGroup sequence =
        new SteppableCommandGroup(
            forwwardTrigger.and(superstructure::atGoal),
            reverseTrigger,

            // Move arm to prepare prepare climb setpoint while moving climbers up
            superstructure.setGoalCommand(Superstructure.Goal.PREPARE_PREPARE_TRAP_CLIMB),

            // Drive forward while raising arm and climber
            drive
                .startEnd(
                    () -> {
                      Pose2d robot = RobotState.getInstance().getEstimatedPose();
                      Pose2d target =
                          robot.transformBy(GeomUtil.toTransform2d(chainToBack.get(), 0.0));
                      drive.setAutoAlignGoal(() -> target, true);
                    },
                    drive::clearAutoAlignGoal)
                .asProxy()
                .until(() -> drive.isAutoAlignGoalCompleted() && superstructure.atGoal())
                .withTimeout(5.0)
                .alongWith(
                    superstructure.setGoalWithConstraintsCommand(
                        Superstructure.Goal.PREPARE_CLIMB,
                        Arm.prepareClimbProfileConstraints.get())),

            // Allow driver to line up and climb
            superstructure
                .setGoalCommand(Superstructure.Goal.CLIMB)
                .alongWith(rollers.setGoalCommand(Rollers.Goal.SHUFFLE_BACKPACK)),

            // Extend backpack
            superstructure.setGoalCommand(Superstructure.Goal.TRAP),

            // Trap.
            superstructure
                .setGoalCommand(Superstructure.Goal.TRAP)
                .alongWith(rollers.setGoalCommand(Rollers.Goal.TRAP_SCORE)),

            // Untrap
            superstructure.setGoalCommand(Superstructure.Goal.UNTRAP));
    return new WrapperCommand(
        sequence
            .beforeStarting(() -> Leds.getInstance().climbing = true)
            .finallyDo(() -> Leds.getInstance().climbing = false)) {
      @Override
      public InterruptionBehavior getInterruptionBehavior() {
        if (sequence.getCurrentCommandIndex().isPresent()
            && sequence.getCurrentCommandIndex().getAsInt() == 0) {
          return InterruptionBehavior.kCancelSelf;
        }
        return InterruptionBehavior.kCancelIncoming;
      }

      @Override
      public String getName() {
        return "Trap Climb";
      }
    };
  }

  public static Command simpleSequence(
      Superstructure superstructure, Trigger forwardTrigger, Trigger reverseTrigger) {
    SteppableCommandGroup sequence =
        new SteppableCommandGroup(
            forwardTrigger.and(superstructure::atGoal),
            reverseTrigger,

            // Raise climber
            superstructure.setGoalCommand(Superstructure.Goal.PREPARE_CLIMB),

            // Climb
            superstructure.setGoalCommand(Superstructure.Goal.CLIMB));
    return new WrapperCommand(
        sequence
            .beforeStarting(() -> Leds.getInstance().climbing = true)
            .finallyDo(() -> Leds.getInstance().climbing = false)) {

      @Override
      public InterruptionBehavior getInterruptionBehavior() {
        if (sequence.getCurrentCommandIndex().isPresent()
            && sequence.getCurrentCommandIndex().getAsInt() == 0) {
          return InterruptionBehavior.kCancelSelf;
        }
        return InterruptionBehavior.kCancelIncoming;
      }

      @Override
      public String getName() {
        return "Simple Climb";
      }
    };
  }
}
