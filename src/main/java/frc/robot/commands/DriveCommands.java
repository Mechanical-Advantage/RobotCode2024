// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive
              .getMotionPlanner()
              .acceptDriveInput(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * DriveConstants.drivetrainConfig.maxLinearVelocity(),
                      linearVelocity.getY() * DriveConstants.drivetrainConfig.maxLinearVelocity(),
                      omega * DriveConstants.drivetrainConfig.maxAngularVelocity(),
                      AllianceFlipUtil.apply(
                          RobotState.getInstance().getEstimatedPose().getRotation())));
        },
        drive);
  }

  public static Command toggleCalculateShotWhileMovingRotation(Drive drive) {
    Supplier<Rotation2d> shotRotation =
        () -> {
          Twist2d fieldVel = RobotState.getInstance().fieldVelocity();
          ShotCalculator.ShotData shotData =
              ShotCalculator.calculate(
                  AllianceFlipUtil.apply(
                      FieldConstants.Speaker.centerSpeakerOpening.getTranslation()),
                  RobotState.getInstance().getEstimatedPose().getTranslation(),
                  new Translation2d(fieldVel.dx, fieldVel.dy));
          return shotData.goalHeading();
        };

    return Commands.runOnce(
        () -> {
          if (drive.getMotionPlanner().isHeadingControlled()) {
            drive.getMotionPlanner().disableHeadingSupplier();
          } else {
            drive.getMotionPlanner().setHeadingSupplier(shotRotation);
          }
        });
  }
}
