// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.frc2024.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
      new Pose2d(startingLineX - 0.5, Stage.podiumLeg.getY(), Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingCenter =
      new Pose2d(
          startingLineX - 0.5,
          StagingLocations.spikeTranslations[1].getY(),
          Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingAmp =
      new Pose2d(
          startingLineX - 0.5,
          StagingLocations.spikeTranslations[2].getY(),
          Rotation2d.fromDegrees(180.0));

  public static final Pose2d startingSourceShifted =
      new Pose2d(startingLineX - 0.5, 4.848, Rotation2d.fromDegrees(180.0));

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
  }

  // Davis 7 Note Auto (named "7note_XXX")
  static {
    final double spikeIntakeOffset = 0.5;
    final double centerlineIntakeOffset = 0.25;
    final double centerlinePrepareIntakeOffset = 1.0;

    Pose2d[] spikeShootingPoses = new Pose2d[3];
    for (int i = 0; i < 3; i++) {
      spikeShootingPoses[i] =
          getShootingPose(StagingLocations.spikeTranslations[i])
              .transformBy(new Translation2d(spikeIntakeOffset, 0.0).toTransform2d());
    }

    final double maxVelocity = 2.0;
    paths.put(
        "7note_spikesWithCenterline4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceShifted)
                // Rotate and shoot podium spike
                .addPoseWaypoint(
                    spikeShootingPoses[0].transformBy(new Translation2d(0.8, 0.0).toTransform2d()))
                .setMaxVelocity(maxVelocity)
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(spikeShootingPoses[0])
                .setStraightLine(true)
                .setMaxOmega(0.0)
                .setMaxVelocity(maxVelocity)
                .build(),
            PathSegment.newBuilder()
                // Intake and shoot spike 1
                .addPoseWaypoint(
                    spikeShootingPoses[1].transformBy(new Translation2d(0.3, 0.4).toTransform2d()))
                .addPoseWaypoint(spikeShootingPoses[1])
                .addPoseWaypoint(
                    spikeShootingPoses[1].transformBy(new Translation2d(-0.3, 0).toTransform2d()))
                .setMaxVelocity(maxVelocity)
                .build(),
            // Intake and shoot spike 2
            PathSegment.newBuilder()
                .addPoseWaypoint(spikeShootingPoses[2])
                .setMaxVelocity(maxVelocity)
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    spikeShootingPoses[2].transformBy(
                        new Translation2d(-spikeIntakeOffset + 0.1, 0.0).toTransform2d()))
                .setStraightLine(true)
                .setMaxOmega(0.0)
                .setMaxVelocity(maxVelocity)
                .build(),
            // Intake and shoot centerline 4
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-175.0))
                        .transformBy(
                            new Translation2d(centerlinePrepareIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-175.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    getShootingPose(
                        new Translation2d(
                            wingX - Units.inchesToMeters(20.0),
                            StagingLocations.centerlineTranslations[4].getY() - 0.75)))
                .build()));
    paths.put(
        "7note_centerline3",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("7note_spikesWithCenterline4"))
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(160.0))
                        .transformBy(
                            new Translation2d(centerlinePrepareIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(160.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    getShootingPose(
                        new Translation2d(
                            wingX - Units.inchesToMeters(20.0),
                            StagingLocations.centerlineTranslations[4].getY() - 1.0)))
                .build()));
    paths.put(
        "7note_centerline2",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("7note_centerline3"))
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(130.0))
                        .transformBy(
                            new Translation2d(centerlinePrepareIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(130.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(130.0))
                        .transformBy(
                            new Translation2d(centerlinePrepareIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    getShootingPose(
                        new Translation2d(
                            wingX - Units.inchesToMeters(20.0),
                            StagingLocations.centerlineTranslations[4].getY() - 1.0)))
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
