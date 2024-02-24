// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.controllers;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.headingControllerConstants;
import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.util.EqualsUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HeadingController {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("HeadingController/kP", headingControllerConstants.kP());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("HeadingController/kD", headingControllerConstants.kD());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "HeadingController/MaxVelocity", headingControllerConstants.maxVelocity());
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "HeadingController/MaxAcceleration", headingControllerConstants.maxAcceleration());
  public static final LoggedTunableNumber slowVelocity =
      new LoggedTunableNumber("HeadingController/SlowVelocity", Math.PI);
  public static final LoggedTunableNumber slowAcceleration =
      new LoggedTunableNumber("HeadingController/SlowAcceleration", 2 * Math.PI);
  public static final LoggedTunableNumber toleranceDegrees =
      new LoggedTunableNumber("HeadingController/ToleranceDegrees", 1.0);

  private final ProfiledPIDController controller;
  private TrapezoidProfile profile;
  private final Supplier<Rotation2d> goalHeadingSupplier;
  private TrapezoidProfile.Constraints maxConstraints;
  private TrapezoidProfile.Constraints slowConstraints;

  // Rejoin trajectory
  private VehicleState rejoinState = null;
  private double totalTrajectoryTime = 0.0;
  private final Timer rejoinTimer = new Timer();

  @AutoLogOutput(key = "Drive/HeadingController/CancellingShot")
  @Getter
  private boolean cancelShot = false;

  public HeadingController(Supplier<Rotation2d> goalHeadingSupplier) {
    controller =
        new ProfiledPIDController(
            kP.get(),
            0,
            kD.get(),
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()),
            Constants.loopPeriodSecs);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
    this.goalHeadingSupplier = goalHeadingSupplier;

    maxConstraints = new TrapezoidProfile.Constraints(slowVelocity.get(), slowAcceleration.get());
    slowConstraints = new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());

    profile = new TrapezoidProfile(slowConstraints);
    controller.setConstraints(maxConstraints);

    controller.reset(
        RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
        RobotState.getInstance().fieldVelocity().dtheta);
  }

  public void setRejoinGoal(HolonomicTrajectory shootingTrajectory) {
    rejoinState = shootingTrajectory.getEndState();
    totalTrajectoryTime = shootingTrajectory.getDuration();
    rejoinTimer.restart();
  }

  /** Returns the rotation rate to turn to aim at speaker */
  public double update() {
    // Update controller
    controller.setPID(kP.get(), 0, kD.get());
    controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          maxConstraints =
              new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
          if (!cancelShot) {
            controller.setConstraints(maxConstraints);
          }
        },
        maxVelocity,
        maxAcceleration);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          slowConstraints =
              new TrapezoidProfile.Constraints(slowVelocity.get(), slowAcceleration.get());
          profile = new TrapezoidProfile(slowConstraints);
        },
        maxVelocity,
        maxAcceleration);

    double output = 0;
    // Calculate output
    if (!cancelShot) {
      output =
          controller.calculate(
              RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
              goalHeadingSupplier.get().getRadians());

      // Check for rejoin
      if (rejoinState != null) {
        // Set profile
        double currentPosition =
            RobotState.getInstance().getEstimatedPose().getRotation().getRadians();
        double endPosition = rejoinState.getTheta();
        double minDistanceToEnd = MathUtil.angleModulus(endPosition - currentPosition);
        profile.calculate(
            Constants.loopPeriodSecs,
            controller.getSetpoint(),
            new TrapezoidProfile.State(endPosition + minDistanceToEnd, rejoinState.getOmega()));
        cancelShot = profile.totalTime() >= totalTrajectoryTime - rejoinTimer.get();
      }
    }

    // Calculate
    if (cancelShot) {
      controller.setConstraints(slowConstraints);
      output =
          controller.calculate(
              RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
              new TrapezoidProfile.State(rejoinState.getTheta(), rejoinState.getOmega()));
    }

    Logger.recordOutput("Drive/HeadingController/Output", output);
    Logger.recordOutput(
        "Drive/HeadingController/SetpointVelocity", controller.getSetpoint().velocity);
    return output;
  }

  /** Returns true if within tolerance of goal heading */
  @AutoLogOutput(key = "Drive/HeadingController/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        controller.getSetpoint().position,
        controller.getGoal().position,
        Units.degreesToRadians(toleranceDegrees.get()));
  }

  @AutoLogOutput(key = "Drive/HeadingController/ThetaError")
  public double getHeadingError() {
    return controller.getPositionError();
  }
}
