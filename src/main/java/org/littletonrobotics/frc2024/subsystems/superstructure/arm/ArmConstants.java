// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2024.Constants;

public class ArmConstants {
  // reduction is 12:62 18:60 12:65
  public static double reduction = (62.0 / 12.0) * (60.0 / 18.0) * (65.0 / 12.0);
  public static Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);
  public static Translation2d armOrigin =
      new Translation2d(-Units.inchesToMeters(9.11), Units.inchesToMeters(11.75));
  public static Rotation2d minAngle = Rotation2d.fromDegrees(10.0);
  public static Rotation2d maxAngle = Rotation2d.fromDegrees(110.0);

  public static int leaderID = 25;
  public static int followerID = 26;
  public static int armEncoderID = 42;

  public static boolean leaderInverted = false;
  public static boolean followerInverted = false;

  /** The offset of the arm encoder in rotations. */
  public static double armEncoderOffsetRotations =
      Units.radiansToRotations(1.2747380347329678 + Math.PI / 2.0);

  public static double armLength =
      switch (Constants.getRobot()) {
        case DEVBOT -> Units.inchesToMeters(24.8);
        default -> Units.inchesToMeters(25.866);
      };

  public static Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case DEVBOT -> new Gains(1200, 0.0, 120, 6.22, 0.0, 0.0, 8.12);
        case COMPBOT -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, 10);

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
