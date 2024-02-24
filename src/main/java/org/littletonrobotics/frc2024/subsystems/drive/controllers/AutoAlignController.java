// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class AutoAlignController {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("AutoAlign/drivekP", 6.0);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("AutoAlign/drivekD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("AutoAlign/thetakP", 8.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("AutoAlign/thetakD", 0.0);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber(
          "AutoAlign/controllerLinearTolerance", 0.04);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber(
          "AutoAlign/controllerThetaTolerance", Units.degreesToRadians(2.0));
  private static final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxLinearVelocity", DriveConstants.autoAlignConstants.maxLinearVelocity());
  private static final LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxLinearAcceleration",
          DriveConstants.autoAlignConstants.maxLinearAcceleration());
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularVelocity", DriveConstants.autoAlignConstants.maxAngularVelocity());
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularAcceleration",
          DriveConstants.autoAlignConstants.maxAngularAcceleration());
  private static final LoggedTunableNumber slowLinearVelocity =
      new LoggedTunableNumber("AutoAlign/slowLinearVelocity", 1.5);
  private static final LoggedTunableNumber slowLinearAcceleration =
      new LoggedTunableNumber("AutoAlign/slowLinearAcceleration", 1.0);
  private static final LoggedTunableNumber slowAngularVelocity =
      new LoggedTunableNumber("AutoAlign/slowAngularVelocity", Math.PI / 2.0);
  private static final LoggedTunableNumber slowAngularAcceleration =
      new LoggedTunableNumber("AutoAlign/slowAngularAcceleration", Math.PI);
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutoAlign/ffMinRadius", 0.2);
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutoAlign/ffMaxRadius", 0.8);

  private final Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;
  private final boolean slowMode;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;

  public AutoAlignController(Supplier<Pose2d> poseSupplier, boolean slowMode) {
    this.poseSupplier = poseSupplier;
    targetPose = poseSupplier.get();
    this.slowMode = slowMode;
    // Set up both controllers
    linearController =
        new ProfiledPIDController(
            linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance.get());
    thetaController =
        new ProfiledPIDController(
            thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0, 0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance.get());
    updateConstraints();
    resetControllers();
  }

  private void updateConstraints() {
    if (slowMode) {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(slowLinearVelocity.get(), slowLinearAcceleration.get()));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              slowAngularVelocity.get(), slowAngularAcceleration.get()));
    } else {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(maxLinearVelocity.get(), maxLinearAcceleration.get()));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(maxAngularVelocity.get(), maxAngularAcceleration.get()));
    }
  }

  private void resetControllers() {
    // Reset measurements and velocities
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Pose2d goalPose = poseSupplier.get();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
    // Linear controller will control to 0 so distance is the measurement
    Rotation2d robotToGoalAngle =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double linearVelocity =
        new Translation2d(fieldVelocity.dx, fieldVelocity.dy).rotateBy(robotToGoalAngle).getX();
    System.out.println(robotToGoalAngle);
    System.out.println(linearVelocity);
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()), -linearVelocity / 2.5);
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
  }

  public ChassisSpeeds update() {
    // Update Controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
        thetakP,
        thetakD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> thetaController.setTolerance(thetaTolerance.get()), thetaTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        this::updateConstraints,
        maxLinearVelocity,
        maxLinearAcceleration,
        slowLinearVelocity,
        slowLinearAcceleration,
        maxAngularVelocity,
        maxAngularAcceleration,
        slowAngularVelocity,
        slowAngularAcceleration);

    // Control to setpoint
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    // Check for reset
    if (!targetPose.equals(poseSupplier.get())) {
      resetControllers();
      targetPose = poseSupplier.get();
    }

    // Calculate feedback velocities (based on position error).
    double linearErrorAbs = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScalar =
        MathUtil.clamp(
            (linearErrorAbs - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    double linearVelocityScalar =
        linearController.getSetpoint().velocity * ffScalar
            + linearController.calculate(linearErrorAbs, 0.0);
    Rotation2d robotToGoalAngle =
        targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    Translation2d desiredLinearVelocity =
        new Translation2d(-linearVelocityScalar, robotToGoalAngle);

    double angularVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())
            + thetaController.getSetpoint().velocity;

    // Show setpoint pose
    Translation2d setpointTranslation =
        new Pose2d(targetPose.getTranslation(), robotToGoalAngle.rotateBy(new Rotation2d(Math.PI)))
            .transformBy(
                new Translation2d(linearController.getSetpoint().position, 0).toTransform2d())
            .getTranslation();
    Rotation2d setpointRotation = new Rotation2d(thetaController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/Setpoint", new Pose2d(setpointTranslation, setpointRotation));

    if (linearController.atGoal()) {
      desiredLinearVelocity = new Translation2d();
    }
    if (thetaController.atGoal()) {
      angularVelocity = 0;
    }
    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            desiredLinearVelocity.getX(), desiredLinearVelocity.getY(), angularVelocity);
    Logger.recordOutput("AutoAlign/GoalPose", poseSupplier.get());
    Logger.recordOutput("AutoAlign/FieldRelativeSpeeds", fieldRelativeSpeeds);
    Logger.recordOutput("AutoAlign/LinearError", linearController.getPositionError());
    Logger.recordOutput("AutoAlign/RotationError", thetaController.getPositionError());
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    return linearController.atGoal() && thetaController.atGoal();
  }
}
