// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAimController {
  private static final LoggedTunableNumber headingkP =
      new LoggedTunableNumber("AutoAim/kP", DriveConstants.headingControllerConstants.kP());
  private static final LoggedTunableNumber headingkD =
      new LoggedTunableNumber("AutoAim/kD", DriveConstants.headingControllerConstants.kD());
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("AutoAim/ToleranceDegrees", 4.0);
  private final PIDController headingController;

  public AutoAimController() {
    headingController = new PIDController(0, 0, 0, 0.02);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Returns the rotation rate to turn to aim at speaker */
  public double update() {
    headingController.setPID(headingkP.get(), 0, headingkD.get());
    headingController.setTolerance(Units.degreesToRadians(tolerance.get()));

    var aimingParams = RobotState.getInstance().getAimingParameters();
    double output =
        headingController.calculate(
            RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
            aimingParams.driveHeading().getRadians());

    Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
    return output;
  }

  /** Returns true if within tolerance of aiming at speaker */
  @AutoLogOutput(key = "AutoAim/AtSetpoint")
  public boolean atSetpoint() {
    return headingController.atSetpoint();
  }
}
