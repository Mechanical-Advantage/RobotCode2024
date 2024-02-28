// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.controllers;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.headingControllerConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2024.util.EqualsUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.frc2024.util.swerve.ModuleLimits;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HeadingController {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("HeadingController/kP", headingControllerConstants.kP());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("HeadingController/kD", headingControllerConstants.kD());
  private static final LoggedTunableNumber maxVelocityMultipler =
      new LoggedTunableNumber("HeadingController/MaxVelocityMultipler", 0.8);
  private static final LoggedTunableNumber maxAccelerationMultipler =
      new LoggedTunableNumber("HeadingController/MaxAccelerationMultipler", 0.8);
  private static final LoggedTunableNumber toleranceDegrees =
      new LoggedTunableNumber("HeadingController/ToleranceDegrees", 1.0);

  private final ProfiledPIDController controller;
  private final Supplier<Rotation2d> goalHeadingSupplier;

  public HeadingController(Supplier<Rotation2d> goalHeadingSupplier) {
    controller =
        new ProfiledPIDController(
            kP.get(),
            0,
            kD.get(),
            new TrapezoidProfile.Constraints(0.0, 0.0),
            Constants.loopPeriodSecs);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
    this.goalHeadingSupplier = goalHeadingSupplier;

    controller.reset(
        RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
        RobotState.getInstance().fieldVelocity().dtheta);
  }

  /** Returns the rotation rate to turn to aim at speaker */
  public double update() {
    // Update controller
    controller.setPID(kP.get(), 0, kD.get());
    controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

    ModuleLimits moduleLimits = RobotState.getInstance().getModuleLimits();
    double maxAngularAcceleration =
        moduleLimits.maxDriveAcceleration()
            / DriveConstants.driveConfig.driveBaseRadius()
            * maxAccelerationMultipler.get();
    double maxAngularVelocity =
        moduleLimits.maxDriveVelocity()
            / DriveConstants.driveConfig.driveBaseRadius()
            * maxVelocityMultipler.get();
    controller.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));

    var output =
        controller.calculate(
            RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
            goalHeadingSupplier.get().getRadians());

    Logger.recordOutput("Drive/HeadingController/HeadingError", controller.getPositionError());
    return output;
  }

  /** Returns true if within tolerance of aiming at speaker */
  @AutoLogOutput(key = "Drive/HeadingController/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        controller.getSetpoint().position,
        controller.getGoal().position,
        Units.degreesToRadians(toleranceDegrees.get()));
  }
}
