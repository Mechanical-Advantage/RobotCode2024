// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(651.223);
  public static final double fieldWidth = Units.inchesToMeters(323.277);
  public static final double wingX = Units.inchesToMeters(229.201);
  public static final double podiumX = Units.inchesToMeters(126.75);
  public static final double startingLineX = Units.inchesToMeters(74.111);

  public static final Translation2d ampCenter =
      new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static final double centerlineX = fieldLength / 2.0;

    // need to update
    public static final double centerlineFirstY = Units.inchesToMeters(29.638);
    public static final double centerlineSeparationY = Units.inchesToMeters(66);
    public static final double spikeX = Units.inchesToMeters(114);
    // need
    public static final double spikeFirstY = Units.inchesToMeters(161.638);
    public static final double spikeSeparationY = Units.inchesToMeters(57);

    public static final Translation2d[] centerlineTranslations = new Translation2d[5];
    public static final Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class Speaker {

    // corners (blue alliance origin)
    public static final Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));

    public static final Translation3d topLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d bottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d centerSpeakerOpening =
        bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
  }

  public static final class Subwoofer {
    public static final Pose2d ampFaceCorner =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(239.366),
            Rotation2d.fromDegrees(-120));

    public static final Pose2d sourceFaceCorner =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(197.466),
            Rotation2d.fromDegrees(120));

    public static final Pose2d centerFace =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(218.416),
            Rotation2d.fromDegrees(180));
  }

  public static final class Stage {
    public static final Pose2d podiumLeg =
        new Pose2d(Units.inchesToMeters(126.75), Units.inchesToMeters(161.638), new Rotation2d());

    public static final Pose2d ampLeg =
        new Pose2d(
            Units.inchesToMeters(220.873),
            Units.inchesToMeters(212.425),
            Rotation2d.fromDegrees(-30));

    public static final Pose2d sourceLeg =
        new Pose2d(
            Units.inchesToMeters(220.873),
            Units.inchesToMeters(110.837),
            Rotation2d.fromDegrees(30));

    public static final Pose2d center =
        new Pose2d(Units.inchesToMeters(192.55), Units.inchesToMeters(161.638), new Rotation2d());
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
