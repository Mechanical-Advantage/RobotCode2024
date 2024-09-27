// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.Robot;
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
  private static final LoggedTunableNumber autoUntrapTime =
      new LoggedTunableNumber("ClimbingCommands/AutoUntrapTime", 134.5);
  private static final LoggedTunableNumber prepareClimbDriveTimeout =
      new LoggedTunableNumber("ClimbingCommands/PrepareClimbDriveTimeout", 0.7);

  // Which mode to use for trap scoring. Can be changed between jackhammering and regular trap
  // scoring.
  private static final Rollers.Goal TRAP_SCORE_ROLLER_GOAL = Rollers.Goal.JACKHAMMERING;

  public static Command trapSequence(
      Drive drive,
      Superstructure superstructure,
      Rollers rollers,
      Trigger forwardTrigger,
      Trigger reverseTrigger) {
    Trigger endOfMatch = Robot.createTeleopTimeTrigger(autoUntrapTime);
    Timer prepareClimbTimer = new Timer();

    SteppableCommandGroup sequence =
        new SteppableCommandGroup(
            forwardTrigger
                .and(superstructure::atArmGoal)
                .or(
                    endOfMatch.and(
                        () ->
                            superstructure.getDesiredGoal() == Superstructure.Goal.TRAP
                                && rollers.getGoal() == Rollers.Goal.TRAP_SCORE)),
            reverseTrigger,

            // Move arm to prepare prepare climb setpoint while moving climbers up
            superstructure.setGoalCommand(Superstructure.Goal.PREPARE_PREPARE_TRAP_CLIMB),

            // Drive forward while raising arm and climber
            Commands.runOnce(prepareClimbTimer::restart)
                .andThen(
                    superstructure
                        .setGoalWithConstraintsCommand(
                            Superstructure.Goal.PREPARE_CLIMB,
                            Arm.prepareClimbProfileConstraints.get())
                        .alongWith(rollers.setGoalCommand(Rollers.Goal.SHUFFLE_BACKPACK))
                        .deadlineWith(
                            drive
                                .startEnd(
                                    () -> {
                                      Pose2d robot = RobotState.getInstance().getEstimatedPose();
                                      Pose2d target =
                                          robot.transformBy(
                                              GeomUtil.toTransform2d(chainToBack.get(), 0.0));
                                      drive.setAutoAlignGoal(
                                          () -> target, Translation2d::new, true);
                                    },
                                    drive::clearAutoAlignGoal)
                                .until(
                                    () ->
                                        prepareClimbTimer.hasElapsed(
                                            prepareClimbDriveTimeout.get()))
                                .andThen(
                                    Commands.runOnce(
                                        () ->
                                            drive.setCoastRequest(Drive.CoastRequest.ALWAYS_COAST)))
                                .asProxy()))
                .finallyDo(() -> drive.setCoastRequest(Drive.CoastRequest.AUTOMATIC)),

            // Pre-move arm to climb position to help with alignment and prevent wedging
            superstructure.setGoalCommand(Superstructure.Goal.POST_PREPARE_TRAP_CLIMB),

            // Allow driver to line up and climb
            superstructure.setGoalCommand(Superstructure.Goal.CLIMB),

            // Extend backpack
            superstructure
                .setGoalCommand(Superstructure.Goal.TRAP)
                .alongWith(rollers.setGoalCommand(Rollers.Goal.TRAP_PRESCORE)),

            // Trap.
            superstructure
                .setGoalCommand(Superstructure.Goal.TRAP)
                .alongWith(rollers.setGoalCommand(TRAP_SCORE_ROLLER_GOAL)),

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
