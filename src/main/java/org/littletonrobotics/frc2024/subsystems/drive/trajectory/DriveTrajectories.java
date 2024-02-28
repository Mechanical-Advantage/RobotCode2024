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
import edu.wpi.first.math.util.Units;
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
  public static final Pose2d startingCenterLine =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.Stage.podiumLeg.getY(),
          Rotation2d.fromDegrees(180.0));

  static {
    // Davis Ethical Auto (5 Note)
    Pose2d stageRightShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(0.5, -1.3)));
    Pose2d stageLeftShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg
                .getTranslation()
                .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.5)
                .plus(new Translation2d(-0.15, 0.2)));
    Translation2d stageRightAvoidance =
        FieldConstants.Stage.sourceLeg.getTranslation().plus(new Translation2d(0.0, -1.2));
    Translation2d stageCenterAvoidance =
        FieldConstants.Stage.sourceLeg
            .getTranslation()
            .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.62);
    paths.put(
        "davisEthicalAuto_grabCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(startingCenterLine.getTranslation()))
                .addTranslationWaypoint(
                    FieldConstants.Stage.podiumLeg
                        .getTranslation()
                        .plus(new Translation2d(0.5, -2.0)))
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
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(160.0))
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
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

    // Unethical auto
    double intakeMaxVelocity = Units.feetToMeters(5.0);
    Pose2d movingShotCenter =
        getShootingPose(
            FieldConstants.StagingLocations.spikeTranslations[1].interpolate(
                FieldConstants.StagingLocations.spikeTranslations[2], 0.5));
    //    paths.put(
    //        "unethicalAuto_grabAll",
    //        List.of(
    //            PathSegment.newBuilder()
    //                .addPoseWaypoint(startingCenterFace)
    //                .addPoseWaypoint(
    //                    movingShotCenter.transformBy(new Translation2d(0.25,
    // 0.0).toTransform2d()))
    //                .build(),
    //            PathSegment.newBuilder()
    //                .addPoseWaypoint(
    //                    movingShotCenter.transformBy(new Translation2d(-0.25,
    // 0.0).toTransform2d()))
    //                .setMaxOmega(0.0)
    //                .setStraightLine(true)
    //                .build(),
    //            PathSegment.newBuilder()
    //                .addPoseWaypoint(
    //                    new Pose2d(
    //                        FieldConstants.StagingLocations.centerlineTranslations[4].plus(
    //                            new Translation2d(-1.0, 0.0)),
    //                        Rotation2d.fromDegrees(180.0)))
    //                .build(),
    //            PathSegment.newBuilder()
    //                .addPoseWaypoint(
    //                    new Pose2d(
    //                        FieldConstants.StagingLocations.centerlineTranslations[4].plus(
    //                            new Translation2d(-0.3, -0.1)),
    //                        Rotation2d.fromDegrees(180.0)))
    //                .setMaxOmega(0.0)
    //                .build(),
    //            PathSegment.newBuilder()
    //                .addPoseWaypoint(
    //                    new Pose2d(
    //                        FieldConstants.StagingLocations.centerlineTranslations[3].plus(
    //                            new Translation2d(-0.3, 0.4)),
    //                        Rotation2d.fromDegrees(135.0)))
    //                .setStraightLine(true)
    //                .setMaxVelocity(intakeMaxVelocity)
    //                .build(),
    //            PathSegment.newBuilder()
    //                .addPoseWaypoint(
    //                    new Pose2d(
    //                        FieldConstants.StagingLocations.centerlineTranslations[0].plus(
    //                            new Translation2d(-0.3, 0.4)),
    //                        Rotation2d.fromDegrees(135.0)))
    //                .setMaxOmega(0.0)
    //                .setStraightLine(true)
    //                .setMaxVelocity(intakeMaxVelocity)
    //                .build(),
    //            PathSegment.newBuilder()
    //                .addTranslationWaypoint(
    //                    FieldConstants.StagingLocations.centerlineTranslations[0].plus(
    //                        new Translation2d(-1.0, 0.0)))
    //                .addTranslationWaypoint(
    //                    FieldConstants.Stage.podiumLeg
    //                        .getTranslation()
    //                        .plus(new Translation2d(-1.5, 0.0)))
    //                .addPoseWaypoint(
    //                    podiumShootingPose.transformBy(new Translation2d(0.65,
    // 0.0).toTransform2d()))
    //                .build()));
    //    paths.put(
    //        "unethicalAuto_grabPodium",
    //        List.of(
    //            PathSegment.newBuilder()
    //                .addWaypoints(getLastWaypoint("unethicalAuto_grabAll"))
    //                .addPoseWaypoint(podiumShootingPose)
    //                .build()));
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
