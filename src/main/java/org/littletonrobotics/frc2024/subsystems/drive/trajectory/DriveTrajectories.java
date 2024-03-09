// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import edu.wpi.first.math.MathUtil;
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
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.PathSegment;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.Waypoint;

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
  public static final Pose2d startingLinePodium =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.Stage.podiumLeg.getY(),
          Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingLineSpike2 =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.StagingLocations.spikeTranslations[2].getY(),
          Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingLineSpike12 =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          (FieldConstants.StagingLocations.spikeTranslations[1].getY()
                  + FieldConstants.StagingLocations.spikeTranslations[2].getY())
              / 2.0,
          Rotation2d.fromDegrees(180.0));

  // Shooting positions
  public static final Pose2d spike0ShootingPose =
      getShootingPose(FieldConstants.StagingLocations.spikeTranslations[0]);
  public static final Pose2d spike1ShootingPose =
      getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1]);
  public static final Pose2d spike2ShootingPose =
      getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2]);

  static {
    // Davis Ethical Auto (4 Note)
    Pose2d stageRightShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(0.5, -1.3)));
    Pose2d stageCenterShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg
                .getTranslation()
                .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.5)
                .plus(new Translation2d(-0.3, 0.2)));
    Translation2d stageRightAvoidance =
        FieldConstants.Stage.sourceLeg.getTranslation().plus(new Translation2d(0.0, -1.2));
    Translation2d stageCenterAvoidance =
        FieldConstants.Stage.sourceLeg
            .getTranslation()
            .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.62);
    Translation2d wingLeftAvoidance =
        new Translation2d(
            FieldConstants.wingX,
            MathUtil.interpolate(
                FieldConstants.Stage.ampLeg.getY(), FieldConstants.fieldWidth, 0.5));

    paths.put(
        "davisEthicalAuto_grabCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(startingLinePodium.getTranslation()))
                .addTranslationWaypoint(
                    FieldConstants.Stage.podiumLeg
                        .getTranslation()
                        .plus(new Translation2d(0.5, -2.0)))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[0],
                            Rotation2d.fromDegrees(170.0))
                        .transformBy(new Translation2d(0.25, -0.07).toTransform2d()))
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
                        .transformBy(new Translation2d(0.15, -0.05).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(160.0))
                        .transformBy(new Translation2d(0.15, -0.05).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageCenterShootingPose)
                .build()));
    paths.put(
        "davisEthicalAuto_grabCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageCenterShootingPose)
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(180.0))
                        .transformBy(new Translation2d(0.3, -0.05).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageCenterShootingPose)
                .build()));

    // Davis Alternative Auto (5 Note)
    Pose2d stageLeftShootingPose =
        getShootingPose(
            FieldConstants.Stage.ampLeg.getTranslation().plus(new Translation2d(-0.7, 1.15)));
    paths.put(
        "davisAlternativeAuto_grabSpike",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(startingLineSpike2.getTranslation()))
                .addPoseWaypoint(
                    spike2ShootingPose.transformBy(new Translation2d(0.5, 0).toTransform2d()))
                .addPoseWaypoint(spike2ShootingPose)
                .build()));
    paths.put(
        "davisAlternativeAuto_grabCenterline4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(spike2ShootingPose)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-135.0))
                        .transformBy(new Transform2d(1.2, -0.2, Rotation2d.fromDegrees(-20.0))))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-135.0))
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    stageLeftShootingPose.transformBy(new Translation2d(-0.8, 0.0).toTransform2d()))
                .addPoseWaypoint(stageLeftShootingPose)
                .build()));
    paths.put(
        "davisAlternativeAuto_grabCenterline3",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageLeftShootingPose)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(new Translation2d(1.2, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    stageLeftShootingPose.transformBy(new Translation2d(-0.8, 0.0).toTransform2d()))
                .addPoseWaypoint(stageLeftShootingPose)
                .build()));
    paths.put(
        "davisAlternativeAuto_grabCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageLeftShootingPose)
                .addTranslationWaypoint(
                    FieldConstants.Stage.ampLeg.getTranslation().plus(new Translation2d(0.1, 1.0)))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(120.0))
                        .transformBy(new Translation2d(1.2, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(120.0))
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageCenterShootingPose)
                .build()));

    paths.put(
        "sourceFRC6328_grabSpike0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceFace)
                .addPoseWaypoint(
                    spike0ShootingPose.transformBy(new Translation2d(0.8, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    spike0ShootingPose.transformBy(new Translation2d(0.5, 0.0).toTransform2d()))
                .build()));
    paths.put(
        "sourceFRC6328_grabSpike1",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("sourceFRC6328_grabSpike0"))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.spikeTranslations[1],
                            Rotation2d.fromDegrees(-160.0))
                        .transformBy(new Translation2d(0.4, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1])
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .build()));
    paths.put(
        "sourceFRC6328_grabSpike2",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("sourceFRC6328_grabSpike1"))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.spikeTranslations[2],
                            Rotation2d.fromDegrees(-130.0))
                        .transformBy(new Translation2d(0.4, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2])
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .build()));
    paths.put(
        "sourceFRC6328_grabCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("sourceFRC6328_grabSpike2"))
                .addTranslationWaypoint(wingLeftAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(new Translation2d(1.0, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(new Translation2d(0.3, 0.0).toTransform2d()))
                .addTranslationWaypoint(wingLeftAvoidance)
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.podiumLeg
                            .getTranslation()
                            .plus(new Translation2d(0.5, 2.5))))
                .build()));

    paths.put(
        "centerSpike_grabSpike0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingCenterFace)
                .addPoseWaypoint(
                    spike0ShootingPose.transformBy(new Translation2d(0.8, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    spike0ShootingPose.transformBy(new Translation2d(0.5, 0.0).toTransform2d()))
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

  public static Waypoint getLastWaypoint(String trajectoryName) {
    List<PathSegment> trajectory = paths.get(trajectoryName);
    return trajectory
        .get(trajectory.size() - 1)
        .getWaypoints(trajectory.get(trajectory.size() - 1).getWaypointsCount() - 1);
  }

  private DriveTrajectories() {}
}
