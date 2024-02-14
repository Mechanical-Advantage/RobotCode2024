// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class DriveTrajectories {
  public static final Map<String, List<PathSegment>> paths = new HashMap<>();

  // Starting locations
  private static final Pose2d startingAmpFace =
      FieldConstants.Subwoofer.ampFaceCorner.transformBy(
          new Transform2d(
              -DriveConstants.driveConfig.bumperWidthX() / 2,
              -DriveConstants.driveConfig.bumperWidthY() / 2,
              new Rotation2d()));
  private static final Pose2d startingSourceFace =
      FieldConstants.Subwoofer.sourceFaceCorner.transformBy(
          new Transform2d(
              -DriveConstants.driveConfig.bumperWidthX() / 2,
              DriveConstants.driveConfig.bumperWidthY() / 2,
              new Rotation2d()));
  private static final Pose2d startingCenterFace =
      FieldConstants.Subwoofer.centerFace.transformBy(
          new Transform2d(DriveConstants.driveConfig.bumperWidthX() / 2, 0, new Rotation2d(0)));

  // Center intake locations
  private static final double intakeOffset = 0.5;
  private static final Pose2d[] intakingCenterlinePoses = new Pose2d[5];

  static {
    // Find locations for intaking centerline gamepieces
    for (int i = 0; i < 5; i++) {
      Translation2d centerLineTranslation =
          FieldConstants.StagingLocations.centerlineTranslations[i];
      intakingCenterlinePoses[i] =
          new Pose2d(
              centerLineTranslation.minus(new Translation2d(intakeOffset, 0)),
              new Rotation2d(Math.PI));
    }

    // Davis Ethical Auto (5 Note)
    paths.put(
        "davisEthicalAuto_driveToCenterline4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceFace)
                .addTranslationWaypoint(new Translation2d(3.5, 2.5))
                .addPoseWaypoint(intakingCenterlinePoses[0])
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .build()));

    paths.put(
        "davisEthicalAuto_driveToCenterline3",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .addTranslationWaypoint(new Translation2d(5.5, 1.4))
                .addPoseWaypoint(intakingCenterlinePoses[1])
                .addTranslationWaypoint(new Translation2d(5.5, 1.4))
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .build()));
    paths.put(
        "davisEthicalAuto_driveToCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .addTranslationWaypoint(
                    FieldConstants.Stage.center
                        .transformBy(new Transform2d(1.25, 0, Rotation2d.fromDegrees(-60)))
                        .getTranslation())
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .addTranslationWaypoint(
                    FieldConstants.Stage.center
                        .transformBy(new Transform2d(1.25, 0, Rotation2d.fromDegrees(-60)))
                        .getTranslation())
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .build()));
    paths.put(
        "davisEthicalAuto_driveToPodium",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .addTranslationWaypoint(new Translation2d(2, 3.75))
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[0])
                        .transformBy(new Transform2d(intakeOffset, 0, new Rotation2d())))
                .build()));

    // 5N-S0-C0123
    paths.put(
        "N5-S0-C0123_driveToS0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmpFace)
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2]))
                .build()));

    paths.put(
        "N5-S0-C0123_driveToC0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2]))
                .addPoseWaypoint(intakingCenterlinePoses[4])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(2, 0, new Rotation2d(Math.PI / 2)))
                            .getTranslation()))
                .build()));

    paths.put(
        "N5-S0-C0123_driveToC1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(2, 0, new Rotation2d(Math.PI / 2)))
                            .getTranslation()))
                .addPoseWaypoint(intakingCenterlinePoses[3])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(2, 0, new Rotation2d(Math.PI / 2)))
                            .getTranslation()))
                .build()));

    paths.put(
        "N5-S0-C0123_driveToC2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(2, 0, new Rotation2d(Math.PI / 2)))
                            .getTranslation()))
                .addTranslationWaypoint(
                    FieldConstants.Stage.center
                        .transformBy(new Transform2d(1.25, 0, Rotation2d.fromDegrees(-60)))
                        .getTranslation())
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .addTranslationWaypoint(
                    FieldConstants.Stage.center
                        .transformBy(new Transform2d(1.25, 0, Rotation2d.fromDegrees(-60)))
                        .getTranslation())
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(2, 0, new Rotation2d(Math.PI / 2)))
                            .getTranslation()))
                .build()));
  }

  // calculate Pose2d of robot given a translation
  private static Pose2d getShootingPose(Translation2d translation) {
    return new Pose2d(
        translation,
        FieldConstants.Speaker.centerSpeakerOpening
            .toTranslation2d()
            .minus(translation)
            .getAngle());
  }

  private DriveTrajectories() {}
}
