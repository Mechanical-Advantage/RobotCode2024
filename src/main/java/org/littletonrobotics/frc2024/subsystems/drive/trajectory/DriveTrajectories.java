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
import org.littletonrobotics.frc2024.util.GeomUtil;

@ExtensionMethod({TrajectoryGenerationHelpers.class, GeomUtil.class})
public class DriveTrajectories {
  public static final Map<String, List<PathSegment>> paths = new HashMap<>();

  // Starting locations
  public static final Pose2d startingAmpFace =
      FieldConstants.Subwoofer.ampFaceCorner.transformBy(
          new Transform2d(
              -DriveConstants.driveConfig.bumperWidthX() / 2,
              -DriveConstants.driveConfig.bumperWidthY() / 2,
              new Rotation2d()));
  public static final Pose2d startingSourceFace =
      FieldConstants.Subwoofer.sourceFaceCorner.transformBy(
          new Transform2d(
              -DriveConstants.driveConfig.bumperWidthX() / 2,
              DriveConstants.driveConfig.bumperWidthY() / 2,
              new Rotation2d()));
  public static final Pose2d startingCenterFace =
      FieldConstants.Subwoofer.centerFace.transformBy(
          new Transform2d(-DriveConstants.driveConfig.bumperWidthX() / 2, 0, new Rotation2d(0)));
  public static final Pose2d startingCenterLine =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.Stage.podiumLeg.getY() + 0.5,
          Rotation2d.fromDegrees(180.0));

  static {
    // Davis Ethical Auto (5 Note)
    Pose2d podiumShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg
                .transformBy(new Translation2d(-0.7, 0.2).toTransform2d())
                .getTranslation());
    Pose2d stageRightShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(0.8, -1.6)));
    Pose2d stageLeftShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg
                .getTranslation()
                .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.5)
                .plus(new Translation2d(-0.1, 0.2)));
    Translation2d stageRightAvoidance =
        FieldConstants.Stage.sourceLeg.getTranslation().plus(new Translation2d(0.0, -1.2));
    Translation2d stageCenterAvoidance =
        FieldConstants.Stage.sourceLeg
            .getTranslation()
            .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.55);
    paths.put(
        "davisEthicalAuto_driveToPodium",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingCenterLine)
                .addPoseWaypoint(
                    podiumShootingPose.transformBy(new Translation2d(0.5, 0.0).toTransform2d()))
                .addPoseWaypoint(podiumShootingPose)
                .build()));
    paths.put(
        "davisEthicalAuto_grabCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(podiumShootingPose)
                .addTranslationWaypoint(
                    podiumShootingPose.getTranslation().plus(new Translation2d(0.2, -1.0)))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[0],
                            Rotation2d.fromDegrees(170.0))
                        .transformBy(new Translation2d(0.5, 0.0).toTransform2d()))
                .addTranslationWaypoint(stageRightAvoidance)
                .addPoseWaypoint(stageRightShootingPose)
                .build()));
    paths.put(
        "davisEthicalAuto_grabCenterline1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageRightShootingPose)
                .addTranslationWaypoint(stageRightAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(-160.0))
                        .transformBy(new Translation2d(0.5, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(160.0))
                        .transformBy(new Translation2d(0.5, 0.0).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageLeftShootingPose)
                .build()));

    paths.put(
        "davisEthicalAuto_grabCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageLeftShootingPose)
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(180.0))
                        .transformBy(new Translation2d(0.5, 0.0).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageLeftShootingPose)
                .build()));

    // paths.put(
    //     "davisEthicalAuto_driveToPodium",
    //     List.of(
    //         PathSegment.newBuilder()
    //             .addPoseWaypoint(
    //                 getShootingPose(
    //                         new Translation2d(
    //                             FieldConstants.startingLineX,
    //                             FieldConstants.StagingLocations.spikeTranslations[0].getY()))
    //                     .transformBy(new Transform2d(doNotHitOffset, 0, new Rotation2d())))
    //             .addPoseWaypoint(
    //                 getShootingPose(FieldConstants.StagingLocations.spikeTranslations[0])
    //                     .transformBy(new Transform2d(doNotHitOffset, 0, new Rotation2d(0))))
    //             .build()));

    // paths.put(
    //     "davisEthicalAuto_driveToCenterline2",
    //     List.of(
    //         PathSegment.newBuilder()
    //             .addWaypoints(getLastWaypoint("davisEthicalAuto_driveToPodium"))
    //             .addPoseWaypoint(
    //                 (FieldConstants.Stage.podiumLeg)
    //                     .transformBy(new Transform2d(0, -.75, new Rotation2d(Math.PI))),
    //                 100)
    //             .addPoseWaypoint(
    //                 new Pose2d(
    //                     FieldConstants.Stage.center.getTranslation(), new Rotation2d(Math.PI)),
    //                 100)
    //             .addPoseWaypoint(intakingApproachCenterlinePoses[2], 100)
    //             .build(),
    //         PathSegment.newBuilder()
    //             .addPoseWaypoint(intakingCenterlinePoses[2], 100)
    //             .setStraightLine(true)
    //             .setMaxOmega(0)
    //             .build(),
    //         PathSegment.newBuilder()
    //             .addPoseWaypoint(
    //                 (FieldConstants.Stage.center)
    //                     .transformBy(new Transform2d(0, -.65, new Rotation2d(Math.PI))),
    //                 100)
    //             .addPoseWaypoint(
    //                 getShootingPose(
    //                     (FieldConstants.Stage.podiumLeg)
    //                         .transformBy(new Transform2d(0, -1.5, new Rotation2d(Math.PI)))
    //                         .getTranslation()),
    //                 100)
    //             .build()));
    // paths.put(
    //     "davisEthicalAuto_driveToCenterline1",
    //     List.of(
    //         PathSegment.newBuilder()
    //             .addWaypoints(getLastWaypoint("davisEthicalAuto_driveToCenterline2"))
    //             .addTranslationWaypoint(
    //                 FieldConstants.Stage.center
    //                     .transformBy(new Transform2d(0, -2.1, new Rotation2d()))
    //                     .getTranslation())
    //             .build(),
    //         PathSegment.newBuilder()
    //             .addPoseWaypoint(intakingCenterlinePoses[1])
    //             .setStraightLine(true)
    //             .setMaxOmega(0)
    //             .build(),
    //         PathSegment.newBuilder()
    //             .addTranslationWaypoint(
    //                 FieldConstants.Stage.center
    //                     .transformBy(new Transform2d(0, -2.1, new Rotation2d()))
    //                     .getTranslation())
    //             .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
    //             .build()));
    // paths.put(
    //     "davisEthicalAuto_driveToCenterline0",
    //     List.of(
    //         PathSegment.newBuilder()
    //             .addWaypoints(getLastWaypoint("davisEthicalAuto_driveToCenterline1"))
    //             .addPoseWaypoint(intakingApproachCenterlinePoses[0])
    //             .build(),
    //         PathSegment.newBuilder()
    //             .addPoseWaypoint(intakingCenterlinePoses[0], 150)
    //             .setStraightLine(true)
    //             .setMaxOmega(0)
    //             .build(),
    //         PathSegment.newBuilder()
    //             .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
    //             .build()));
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

  public static Waypoint getLastWaypoint(String trajectoryName) {
    List<PathSegment> trajectory = paths.get(trajectoryName);
    return trajectory
        .get(trajectory.size() - 1)
        .getWaypoints(trajectory.get(trajectory.size() - 1).getWaypointsCount() - 1);
  }

  private DriveTrajectories() {}
}
