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
        "davisEthicalAuto_driveToPodium",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceFace)
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[0])
                        .transformBy(new Transform2d(intakeOffset, 0, new Rotation2d())))
                .build()));

    paths.put(
        "davisEthicalAuto_driveToCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[0])
                        .transformBy(new Transform2d(intakeOffset, 0, new Rotation2d())))
                .addPoseWaypoint(
                    (FieldConstants.Stage.podiumLeg)
                        .transformBy(new Transform2d(0, -.75, new Rotation2d(Math.PI))))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.Stage.center.getTranslation(), new Rotation2d(Math.PI)))
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .addPoseWaypoint(
                    (FieldConstants.Stage.center)
                        .transformBy(new Transform2d(0, -.65, new Rotation2d(Math.PI))))
                .addPoseWaypoint(
                    getShootingPose(
                        (FieldConstants.Stage.podiumLeg)
                            .transformBy(new Transform2d(0, -.75, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .build()));
    paths.put(
        "davisEthicalAuto_driveToCenterline1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    (FieldConstants.Stage.podiumLeg)
                        .transformBy(new Transform2d(0, -.75, new Rotation2d(Math.PI))))
                .addTranslationWaypoint(
                    FieldConstants.Stage.center
                        .transformBy(new Transform2d(0, -2.1, new Rotation2d()))
                        .getTranslation())
                .addPoseWaypoint(intakingCenterlinePoses[1])
                .addTranslationWaypoint(
                    FieldConstants.Stage.center
                        .transformBy(new Transform2d(0, -2.1, new Rotation2d()))
                        .getTranslation())
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .build()));
    paths.put(
        "davisEthicalAuto_driveToCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .addPoseWaypoint(intakingCenterlinePoses[0])
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .build()));

    // N5-S1-C234

    paths.put(
        "N5-S1-C234_driveToS1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingCenterFace)
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1]))
                .build()));

    paths.put(
        "N5-S1-C234_driveToC2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1]))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.Stage.center.getTranslation(), new Rotation2d(Math.PI)))
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, .65, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .build()));

    paths.put(
        "N5-S1-C234_driveToC3",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, .65, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .addPoseWaypoint(
                    FieldConstants.Stage.ampLeg.transformBy(
                        new Transform2d(0, -1, new Rotation2d(Math.PI))))
                .addPoseWaypoint(intakingCenterlinePoses[3])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2.1, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .build()));
    paths.put(
        "N5-S1-C234_driveToC4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2.1, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .addPoseWaypoint(intakingCenterlinePoses[4])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2.1, new Rotation2d(Math.PI)))
                            .getTranslation()))
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
                .addPoseWaypoint(
                    new Pose2d(
                        intakingCenterlinePoses[4].getTranslation(), Rotation2d.fromDegrees(-180)))
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 1.3, new Rotation2d()))
                            .getTranslation()),
                    30)
                .build()));

    paths.put(
        "N5-S0-C0123_driveToC1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 1.3, new Rotation2d()))
                            .getTranslation()))
                .addPoseWaypoint(
                    new Pose2d(
                        intakingCenterlinePoses[3].getTranslation(), Rotation2d.fromDegrees(-180)))
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

    // N6-S12-C0123
    paths.put(
        "N6-S12-C0123_driveToS1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmpFace)
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1]))
                .build()));

    paths.put(
        "N6-S12-C0123_driveToC0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1]))
                .addPoseWaypoint(
                    new Pose2d(
                        intakingCenterlinePoses[4].getTranslation(),
                        Rotation2d.fromDegrees(-180)))
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 1.3, new Rotation2d()))
                            .getTranslation()),
                    30)
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
