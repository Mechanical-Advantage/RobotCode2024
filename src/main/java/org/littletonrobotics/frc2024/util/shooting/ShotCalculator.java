// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.util.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Util class to calculate robot rotation and shooter feedforward values for shooting while moving.
 *
 * <p>Based off of 254 Rapid React code from class <a
 * href="https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java">ShootingUtil.java</a>.
 * See the license file in the root directory of this project.
 */
@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  public record ShotData(
      double effectiveRobotToSpeakerDist,
      double radialFeedforward, // ff value due to radial velocity of robot to speaker
      Rotation2d goalHeading) {} // heading of robot to match tangential velocity

  /** In theory we will aim at different locations inside speaker */
  public static ShotData calculate(
      Translation2d speaker, Translation2d robot, Translation2d linearFieldVelocity) {
    // Calculate radial and tangential velocity from speaker
    Rotation2d speakerToRobotAngle = robot.minus(speaker).getAngle();
    Translation2d tangentialVelocity =
        linearFieldVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
    // Positive when velocity is away from speaker
    double radialComponent = tangentialVelocity.getX();
    // Positive when traveling CCW about speaker
    double tangentialComponent = tangentialVelocity.getY();

    // TODO: what does this do
    // Ig this is the estimated time of the note in the air
    // later on this will be a function of the distance
    final double shotTime = 1.05;

    // Add robot velocity to raw shot speed
    double rawDistToGoal = robot.getDistance(speaker);
    double shotSpeed = rawDistToGoal / shotTime + radialComponent;
    if (shotSpeed <= 0.0) shotSpeed = 0.0;
    // Rotate back into field frame then add take opposite
    Rotation2d goalHeading =
        robot.toPose2d().inverse().transformBy(speaker.toTransform2d()).getTranslation().getAngle();
    // Aim opposite of tangentialComponent (negative lead when tangentialComponent is positive)
    goalHeading = goalHeading.plus(new Rotation2d(shotSpeed, tangentialComponent));
    double effectiveDist = shotTime * Math.hypot(tangentialComponent, shotSpeed);

    Logger.recordOutput("ShootWhileMoving/heading", goalHeading);
    Logger.recordOutput("ShootWhileMoving/driveFeedVelocity", radialComponent);
    Logger.recordOutput("ShootWhileMoving/effectiveDistance", effectiveDist);
    // Use radial component of velocity for ff value
    return new ShotData(effectiveDist, radialComponent, goalHeading);
  }
}
