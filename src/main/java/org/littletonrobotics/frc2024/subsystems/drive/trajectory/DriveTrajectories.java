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
  public static final Pose2d startingFarSource =
      new Pose2d(FieldConstants.startingLineX - 0.5, 1.57, Rotation2d.fromDegrees(180));

  // Davis Spiky Auto (named "spiky_XXX")
  static {
  }

  // Davis Speedy Auto (named "speedy_XXX")
  static {
  }

  // Davis Alternative Speedy Auto (named "speedyAlt_XXX")
  static {
  }

  // Davis Ethical Auto (named "ethical_XXX")
  static {
  }

  // Davis Unethical Auto (named "unethical_XXX")
  static {
    final Pose2d sourceSideShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(0.5, -1.3)));
    final Pose2d sourceSideCloseShootingPose =
        getShootingPose(
            FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(-0.5, -1.8)));
    final Translation2d sourceLegAvoidance =
        new Translation2d(
            FieldConstants.wingX,
            MathUtil.interpolate(0, FieldConstants.Stage.sourceLeg.getY(), 0.63));

    final PathSegment intakeCenterline0 =
        PathSegment.newBuilder()
            .addPoseWaypoint(
                new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[0],
                        Rotation2d.fromDegrees(170))
                    .transformBy(new Transform2d(0.5, 0, new Rotation2d())))
            .addPoseWaypoint(
                new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[0],
                        Rotation2d.fromDegrees(170))
                    .transformBy(new Transform2d(0.3, 0, new Rotation2d())))
            .build();
    final PathSegment intakeCenterline1 =
        PathSegment.newBuilder()
            .addPoseWaypoint(
                new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[1],
                        Rotation2d.fromDegrees(-170))
                    .transformBy(new Transform2d(0.5, 0, new Rotation2d())))
            .addPoseWaypoint(
                new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[1],
                        Rotation2d.fromDegrees(-170))
                    .transformBy(new Transform2d(0.3, 0, new Rotation2d())))
            .build();

    paths.put(
        "unethical_grabCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingFarSource)
                .addPoseWaypoint(
                    startingFarSource.transformBy(new Transform2d(-1.25, 0, new Rotation2d())))
                .setStraightLine(true)
                .setMaxOmega(0)
                .build(),
            intakeCenterline0,
            PathSegment.newBuilder().addPoseWaypoint(sourceSideShootingPose).build()));
    paths.put(
        "unethical_grabCenterline1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingFarSource)
                .addPoseWaypoint(
                    startingFarSource.transformBy(new Transform2d(-1.4, 0, new Rotation2d())))
                .setStraightLine(true)
                .setMaxOmega(0)
                .build(),
            PathSegment.newBuilder().addTranslationWaypoint(sourceLegAvoidance).build(),
            intakeCenterline1,
            PathSegment.newBuilder()
                .addTranslationWaypoint(sourceLegAvoidance)
                .addPoseWaypoint(sourceSideShootingPose)
                .build()));
    paths.put(
        "unethical_centerline0ToCenterline1",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("unethical_grabCenterline0"))
                .addTranslationWaypoint(sourceLegAvoidance)
                .build(),
            intakeCenterline1,
            PathSegment.newBuilder()
                .addTranslationWaypoint(sourceLegAvoidance)
                .addPoseWaypoint(sourceSideCloseShootingPose)
                .build()));
    paths.put(
        "unethical_centerline1ToCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("unethical_grabCenterline1"))
                .build(),
            intakeCenterline0,
            PathSegment.newBuilder().addPoseWaypoint(sourceSideCloseShootingPose).build()));
    paths.put(
        "unethical_grabEjected",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("unethical_centerline1ToCenterline0"))
                .addPoseWaypoint(
                    startingFarSource.transformBy(
                        new Transform2d(-0.5, 0.0, Rotation2d.fromDegrees(180.0))))
                .addPoseWaypoint(
                    startingFarSource.transformBy(
                        new Transform2d(0.5, 0.0, Rotation2d.fromDegrees(180.0))))
                .addPoseWaypoint(sourceSideCloseShootingPose)
                .build()));
    paths.put(
        "unethical_driveToSource",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("unethical_grabEjected"))
                .addPoseWaypoint(
                    new Pose2d(FieldConstants.wingX + 1.0, 1.5, Rotation2d.fromDegrees(180)))
                .build(),
            PathSegment.newBuilder()
                .addTranslationWaypoint(new Translation2d((FieldConstants.fieldLength) - 2, 1))
                .setStraightLine(true)
                .setMaxOmega(0)
                .build()));
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
