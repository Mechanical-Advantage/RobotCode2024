// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.subsystems.drive.Drive;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterization extends Command {
  @RequiredArgsConstructor
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;
  }

  private final Drive drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);
  private final double driveRadius = DriveConstants.driveConfig.driveBaseRadius();

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double averageWheelPosition;
  private double startWheelPosition;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(Drive drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = drive.getGyroYaw().getRadians();
    accumGyroYawRads = 0.0;

    averageWheelPosition =
        Arrays.stream(drive.getWheelRadiusCharacterizationPosition()).sum() / 4.0;
    startWheelPosition = averageWheelPosition;

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drive.runWheelRadiusCharacterization(omegaLimiter.calculate(omegaDirection.value * 3.0));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(drive.getGyroYaw().getRadians() - lastGyroYawRads);
    lastGyroYawRads = drive.getGyroYaw().getRadians();
    averageWheelPosition =
        (Arrays.stream(drive.getWheelRadiusCharacterizationPosition()).sum() / 4.0)
            - startWheelPosition;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    Logger.recordOutput(
        "Drive/EffectiveWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
