// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.AutoSelector;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;

@RequiredArgsConstructor
public class DemoAutos {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Supplier<List<AutoSelector.AutoQuestionResponse>> responses;

  public Command davisDemoAuto() {
    return Commands.either(
        followTag(),
        lookAtTag(),
        () -> responses.get().get(0).equals(AutoSelector.AutoQuestionResponse.YES));
  }

  private Command followTag() {
    return drive
        .startEnd(
            () ->
                drive.setAutoAlignGoal(
                    () ->
                        RobotState.getInstance()
                            .getDemoTagParameters()
                            .map(RobotState.DemoAimingParameters::targetPose)
                            .orElseGet(() -> RobotState.getInstance().getEstimatedPose()),
                    Translation2d::new,
                    false),
            drive::clearAutoAlignGoal)
        .alongWith(
            superstructure.setGoalWithConstraintsCommand(
                Superstructure.Goal.AIM_AT_DEMO_TAG, Arm.smoothProfileConstraints.get()));
  }

  private Command lookAtTag() {
    return drive
        .startEnd(
            () ->
                drive.setHeadingGoal(
                    () ->
                        RobotState.getInstance()
                            .getDemoTagParameters()
                            .map(RobotState.DemoAimingParameters::targetHeading)
                            .orElseGet(
                                () -> RobotState.getInstance().getEstimatedPose().getRotation())),
            drive::clearHeadingGoal)
        .alongWith(
            superstructure.setGoalWithConstraintsCommand(
                Superstructure.Goal.AIM_AT_DEMO_TAG, Arm.smoothProfileConstraints.get()));
  }
}
