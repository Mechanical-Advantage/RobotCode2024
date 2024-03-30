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
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.PathSegment;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.Waypoint;

@ExtensionMethod({TrajectoryGenerationHelpers.class, GeomUtil.class})
public class DriveTrajectories {
  public static final Map<String, List<PathSegment>> paths = new HashMap<>();

  // Starting locations
  public static final Pose2d startingSource =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.Stage.podiumLeg.getY(),
          Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingCenter =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.StagingLocations.spikeTranslations[1].getY(),
          Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingAmp =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.StagingLocations.spikeTranslations[2].getY(),
          Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingAmpWall =
      new Pose2d(
          FieldConstants.startingLineX - 0.5,
          FieldConstants.fieldWidth - 0.5,
          Rotation2d.fromDegrees(180.0));

  // Shooting poses
  public static final Pose2d stageCenterShootingPose =
      getShootingPose(
          FieldConstants.Stage.podiumLeg
              .getTranslation()
              .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.5)
              .plus(new Translation2d(-0.4, 0.2)));
  public static final Pose2d stageRightShootingPose =
      getShootingPose(
          FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(0.5, -1.3)));
  public static final Pose2d stageLeftShootingPose =
      getShootingPose(
          FieldConstants.Stage.ampLeg.getTranslation().plus(new Translation2d(-0.7, 1.15)));

  // Avoidance points
  public static final Translation2d wingLeftAvoidance =
      new Translation2d(
          FieldConstants.wingX,
          MathUtil.interpolate(FieldConstants.Stage.ampLeg.getY(), FieldConstants.fieldWidth, 0.3));
  public static final Translation2d stageRightAvoidance =
      FieldConstants.Stage.sourceLeg.getTranslation().plus(new Translation2d(0.0, -1.2));
  public static final Translation2d stageCenterAvoidance =
      FieldConstants.Stage.sourceLeg
          .getTranslation()
          .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.62);

  // Davis Spiky Auto (named "spiky_XXX")
  static {
  }

  // Davis Speedy Auto (named "speedy_XXX")
  static {
    paths.put(
        "speedy_ampToCenterline4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmpWall)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-160.0))
                        .transformBy(new Transform2d(.3, -0.2, Rotation2d.fromDegrees(-20.0))))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-160.0))
                        .transformBy(new Translation2d(0.3, 0.0).toTransform2d()))
                .addPoseWaypoint(stageLeftShootingPose)
                .build()));
    paths.put(
        "speedy_centerline4ToCenterline3",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("speedy_ampToCenterline4"))
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
                .addTranslationWaypoint(wingLeftAvoidance)
                .addPoseWaypoint(stageLeftShootingPose)
                .build()));
    paths.put(
        "speedy_centerline3ToCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("speedy_centerline4ToCenterline3"))
                .addTranslationWaypoint(wingLeftAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(new Translation2d(1.2, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageCenterShootingPose)
                .build()));
    paths.put(
        "speedy_centerline2ToEjectedNote",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("speedy_centerline3ToCenterline2"))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.Amp.ampTapeTopCorner.plus(new Translation2d(1.3, -0.3)),
                        new Rotation2d(-Math.PI / 2)))
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Amp.ampTapeTopCorner.plus(new Translation2d(1.0, -0.5))))
                .build()));
  }

  // Davis Ethical Auto (named "ethical_XXX")
  static {
  }

  // Davis Unethical Auto (named "unethical_XXX")
  static {
  }

  /** Calculates aimed pose from translation. */
  private static Pose2d getShootingPose(Translation2d translation) {
    return new Pose2d(
        translation,
        FieldConstants.Speaker.centerSpeakerOpening
            .toTranslation2d()
            .minus(translation)
            .getAngle());
  }

  /** Returns the last waypoint of a trajectory. */
  public static Waypoint getLastWaypoint(String trajectoryName) {
    List<PathSegment> trajectory = paths.get(trajectoryName);
    return trajectory
        .get(trajectory.size() - 1)
        .getWaypoints(trajectory.get(trajectory.size() - 1).getWaypointsCount() - 1);
  }

  private DriveTrajectories() {}
}
