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
  public static final double reduction = (62.0 / 12.0) * (60.0 / 18.0) * (65.0 / 12.0);
  public static final Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);
  public static final Translation2d armOrigin = new Translation2d(-0.238, 0.298);
  public static final Rotation2d minAngle =
      switch (Constants.getRobot()) {
        default -> Rotation2d.fromDegrees(0.0);
        case DEVBOT -> Rotation2d.fromDegrees(10.0);
      };
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(110.0);

  public static final int leaderID =
      switch (Constants.getRobot()) {
        default -> 11;
        case DEVBOT -> 25;
      };
  public static final int followerID =
      switch (Constants.getRobot()) {
        default -> 10;
        case DEVBOT -> 26;
      };
  public static final int armEncoderID =
      switch (Constants.getRobot()) {
        default -> 0;
        case DEVBOT -> 42;
      };

  public static final boolean leaderInverted = false;

  /** The offset of the arm encoder in radians. */
  public static final double armEncoderOffsetRads =
      switch (Constants.getRobot()) {
        default -> 1.1980389953386859;
        case DEVBOT -> -1.233 - Math.PI / 2.0;
      };

  public static final double armLength =
      switch (Constants.getRobot()) {
        case DEVBOT -> Units.inchesToMeters(24.8);
        default -> Units.inchesToMeters(25.866);
      };

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case DEVBOT -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(6000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
      };

  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, 15);

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
