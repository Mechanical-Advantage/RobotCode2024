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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
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
      new LoggedTunableNumber("AutoAlign/drivekP", 1.5);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("AutoAlign/drivekD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("AutoAlign/thetakP", 5.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("AutoAlign/thetakD", 0.0);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber("AutoAlign/controllerLinearTolerance", 0.05);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("AutoAlign/controllerThetaTolerance", Units.degreesToRadians(2.0));
  private static final LoggedTunableNumber toleranceTime =
      new LoggedTunableNumber("AutoAlign/controllerToleranceSecs", 0.4);
  private static final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxLinearVelocity", DriveConstants.driveConfig.maxLinearVelocity());
  private static final LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxLinearAcceleration",
          DriveConstants.driveConfig.maxLinearAcceleration() * 0.4);
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularVelocity", DriveConstants.driveConfig.maxAngularVelocity() * 0.8);
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularAcceleration",
          DriveConstants.driveConfig.maxAngularAcceleration() * 0.8);
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
  private final boolean slowMode;
  private Translation2d lastSetpointTranslation;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;
  private final Timer toleranceTimer = new Timer();

  public AutoAlignController(Supplier<Pose2d> poseSupplier, boolean slowMode) {
    this.poseSupplier = poseSupplier;
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
    toleranceTimer.restart();
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
    Rotation2d robotToGoalAngle =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double linearVelocity =
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(robotToGoalAngle.unaryMinus())
                .getX());
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()), linearVelocity);
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
    lastSetpointTranslation = currentPose.getTranslation();
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
    Pose2d targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    linearController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        linearController.getSetpoint().velocity);
    double driveVelocityScalar =
        linearController.getSetpoint().velocity * ffScaler
            + linearController.calculate(currentDistance, 0.0);
    if (linearController.atGoal()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(linearController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Reset tolerance timer
    if (!linearController.atGoal() || !thetaController.atGoal()) {
      toleranceTimer.reset();
    }

    // Log data
    Logger.recordOutput("AutoAlign/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutoAlign/DistanceSetpoint", linearController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/SetpointPose",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/GoalPose", targetPose);

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    return toleranceTimer.hasElapsed(toleranceTime.get());
  }
}
