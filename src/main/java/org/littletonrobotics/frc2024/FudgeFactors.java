// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Builder;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

/** Fudge factors for critical field locations which can be tuned per alliance at events. */
@ExtensionMethod({GeomUtil.class})
public final class FudgeFactors {
  @Builder
  public static class FudgedTransform {
    @Builder.Default private final Transform2d red = new Transform2d();

    @Builder.Default private final Transform2d blue = new Transform2d();

    public Transform2d getTransform() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? red : blue)
          .orElseGet(Transform2d::new);
    }
  }

  private FudgeFactors() {}

  public static final FudgedTransform speaker = FudgedTransform.builder().build();

  public static final FudgedTransform amp = FudgedTransform.builder().build();

  public static final FudgedTransform centerPodiumAmpChain = FudgedTransform.builder().build();

  public static final FudgedTransform centerAmpSourceChain = FudgedTransform.builder().build();

  public static final FudgedTransform centerSourcePodiumChain = FudgedTransform.builder().build();

  /**
   * Flips a translation in the same way {@link org.littletonrobotics.frc2024.util.AllianceFlipUtil}
   * does for the red alliance, used for logging.
   *
   * @param translation The translation to flip
   * @return The flipped translation
   */
  private static Translation2d flipTranslation(Translation2d translation) {
    return translation.withX(FieldConstants.fieldLength - translation.getX());
  }

  /**
   * Flips a pose in the same way {@link org.littletonrobotics.frc2024.util.AllianceFlipUtil} does
   * for the red alliance, used for logging.
   *
   * @param pose The pose to flip
   * @return The flipped pose
   */
  private static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(
        flipTranslation(pose.getTranslation()),
        new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
  }

  public static void logPoses() {
    Logger.recordOutput(
        "FudgeFactors/RedSpeaker",
        flipTranslation(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d())
            .toTransform2d()
            .plus(speaker.red)
            .toPose2d());
    Logger.recordOutput(
        "FudgeFactors/BlueSpeaker",
        FieldConstants.Speaker.centerSpeakerOpening
            .toTranslation2d()
            .toTransform2d()
            .plus(speaker.blue)
            .toPose2d());

    Logger.recordOutput(
        "FudgeFactors/RedAmp",
        flipPose(new Pose2d(FieldConstants.ampCenter, new Rotation2d(-Math.PI / 2.0)))
            .transformBy(GeomUtil.toTransform2d(DriveConstants.robotCenterToIntakeBumperEdge, 0))
            .transformBy(FudgeFactors.amp.red));
    Logger.recordOutput(
        "FudgeFactors/BlueAmp",
        new Pose2d(FieldConstants.ampCenter, new Rotation2d(-Math.PI / 2.0))
            .transformBy(GeomUtil.toTransform2d(DriveConstants.robotCenterToIntakeBumperEdge, 0))
            .transformBy(FudgeFactors.amp.blue));

    Logger.recordOutput(
        "FudgeFactors/RedCenterPodiumAmpChain",
        flipPose(FieldConstants.Stage.centerPodiumAmpChain).transformBy(centerPodiumAmpChain.red));
    Logger.recordOutput(
        "FudgeFactors/BlueCenterPodiumAmpChain",
        FieldConstants.Stage.centerPodiumAmpChain.transformBy(centerPodiumAmpChain.blue));
    Logger.recordOutput(
        "FudgeFactors/RedCenterAmpSourceChain",
        flipPose(FieldConstants.Stage.centerAmpSourceChain).transformBy(centerAmpSourceChain.red));
    Logger.recordOutput(
        "FudgeFactors/BlueCenterAmpSourceChain",
        FieldConstants.Stage.centerAmpSourceChain.transformBy(centerAmpSourceChain.blue));
    Logger.recordOutput(
        "FudgeFactors/RedCenterSourcePodiumChain",
        flipPose(FieldConstants.Stage.centerSourcePodiumChain)
            .transformBy(centerSourcePodiumChain.red));
    Logger.recordOutput(
        "FudgeFactors/BlueCenterSourcePodiumChain",
        FieldConstants.Stage.centerSourcePodiumChain.transformBy(centerSourcePodiumChain.blue));
  }
}
