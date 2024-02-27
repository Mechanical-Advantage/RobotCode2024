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
          new Transform2d(-DriveConstants.driveConfig.bumperWidthX() / 2, 0, new Rotation2d(0)));
  public static final Pose2d startingFacingPodium =
      new Pose2d(
          new Translation2d(
              FieldConstants.startingLineX - Units.inchesToMeters(20),
              FieldConstants.StagingLocations.spikeTranslations[0].getY()),
          new Rotation2d(Math.PI));

  // Center intake locations
  private static final double intakeOffset = 0.4;
  private static final double intakeApproachOffset = .5;
  private static final double doNotHitOffset = 0.3;
  private static final Pose2d[] intakingCenterlinePoses = new Pose2d[5];
  private static final Pose2d[] intakingApproachCenterlinePoses = new Pose2d[5];

  static {
    // Find locations for intaking centerline gamepieces
    for (int i = 0; i < 5; i++) {
      Translation2d centerLineTranslation =
          FieldConstants.StagingLocations.centerlineTranslations[i];
      intakingCenterlinePoses[i] =
          new Pose2d(
              centerLineTranslation.minus(new Translation2d(intakeOffset, 0)),
              new Rotation2d(Math.PI));
      intakingApproachCenterlinePoses[i] =
          new Pose2d(
              centerLineTranslation.minus(new Translation2d(intakeApproachOffset, 0)),
              new Rotation2d(Math.PI));
    }

    // Davis Ethical Auto (5 Note)
    paths.put(
        "davisEthicalAuto_driveToPodium",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(startingFacingPodium.getTranslation()))
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[0])
                        .transformBy(new Transform2d(doNotHitOffset, 0, new Rotation2d(0))))
                .build()));

    paths.put(
        "davisEthicalAuto_driveToCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(startingFacingPodium.getTranslation()))
                .addPoseWaypoint(
                    (FieldConstants.Stage.podiumLeg)
                        .transformBy(new Transform2d(0, -.95, new Rotation2d(Math.PI))))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.Stage.center.getTranslation(), new Rotation2d(Math.PI)))
                .addPoseWaypoint(intakingApproachCenterlinePoses[2])
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .setStraightLine(true)
                .setMaxOmega(0)
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    (FieldConstants.Stage.center)
                        .transformBy(new Transform2d(-.4, -.70, new Rotation2d(Math.PI))))
                .addPoseWaypoint(
                    getShootingPose(
                        (FieldConstants.Stage.podiumLeg)
                            .transformBy(new Transform2d(.2, -1.4, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .build()));
    paths.put(
        "davisEthicalAuto_driveToCenterline1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getLastWaypointPose("davisEthicalAuto_driveToCenterline2"))
                .addTranslationWaypoint(
                    FieldConstants.Stage.center
                        .transformBy(new Transform2d(0, -2.1, new Rotation2d()))
                        .getTranslation())
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(intakingCenterlinePoses[1])
                .setStraightLine(true)
                .setMaxOmega(0)
                .build(),
            PathSegment.newBuilder()
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
                .addPoseWaypoint(getLastWaypointPose("davisEthicalAuto_driveToCenterline1"))
                .addPoseWaypoint(intakingApproachCenterlinePoses[0])
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(intakingCenterlinePoses[0])
                .setStraightLine(true)
                .setMaxOmega(0)
                .build(),
            PathSegment.newBuilder()
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
                .addPoseWaypoint(new Pose2d())
                .addTranslationWaypoint(new Translation2d(3.0, 0.5))
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

    // 5N-S0-C432
    paths.put(
        "N5-S2-C432_driveToS2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmpFace)
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2]))
                .build()));

    paths.put(
        "N5-S2-C432_driveToC4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2]))
                .addPoseWaypoint(intakingCenterlinePoses[4])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2.2, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .build()));

    paths.put(
        "N5-S2-C432_driveToC3",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2.2, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .addPoseWaypoint(intakingCenterlinePoses[3])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2.2, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .build()));

    paths.put(
        "N5-S2-C432_driveToC2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2.2, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .addTranslationWaypoint(
                    FieldConstants.Stage.ampLeg
                        .transformBy(new Transform2d(0, -1, Rotation2d.fromDegrees(60)))
                        .getTranslation())
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.ampLeg
                            .transformBy(new Transform2d(-0.5, -1.25, Rotation2d.fromDegrees(60)))
                            .getTranslation()))
                .build()));

    // N5-C012-S0
    paths.put(
        "N5-C012-S0_driveToC0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceFace)
                .addPoseWaypoint(intakingCenterlinePoses[0])
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .build()));

    paths.put(
        "N5-C012-S0_driveToC1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .addTranslationWaypoint(
                    FieldConstants.Stage.sourceLeg
                        .transformBy(new Transform2d(0, -1, new Rotation2d(0)))
                        .getTranslation())
                .addPoseWaypoint(intakingCenterlinePoses[1])
                .addTranslationWaypoint(
                    FieldConstants.Stage.sourceLeg
                        .transformBy(new Transform2d(0, -1, new Rotation2d()))
                        .getTranslation())
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .build()));
    paths.put(
        "N5-C012-S0_driveToC2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .addTranslationWaypoint(
                    FieldConstants.Stage.sourceLeg
                        .transformBy(new Transform2d(0, 1, Rotation2d.fromDegrees(0)))
                        .getTranslation())
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .addTranslationWaypoint(
                    FieldConstants.Stage.sourceLeg
                        .transformBy(new Transform2d(0, 1, Rotation2d.fromDegrees(0)))
                        .getTranslation())
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .build()));

    paths.put(
        "N5-C012-S0_driveToPodium",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.6)))
                .addTranslationWaypoint(
                    FieldConstants.Stage.podiumLeg
                        .transformBy(new Transform2d(-1, 0, new Rotation2d()))
                        .getTranslation())
                .addPoseWaypoint(
                    getShootingPose(
                        new Pose2d(
                                FieldConstants.StagingLocations.spikeTranslations[0],
                                new Rotation2d(0))
                            .transformBy(new Transform2d(-intakeOffset, 0, new Rotation2d()))
                            .getTranslation()))
                .build()));

    // N6-S12-C0123
    paths.put(
        "N6-S21-C432_driveToS2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmpFace)
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2]))
                .build()));

    paths.put(
        "N6-S21-C432_driveToS1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[2]))
                .addTranslationWaypoint(
                    new Translation2d(
                        FieldConstants.startingLineX,
                        (FieldConstants.StagingLocations.spikeTranslations[2].getY()
                                + FieldConstants.StagingLocations.spikeTranslations[1].getY())
                            / 2))
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1]))
                .build()));

    paths.put(
        "N6-S21-C432_driveToC4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(FieldConstants.StagingLocations.spikeTranslations[1]))
                .addPoseWaypoint(
                    new Pose2d(
                        intakingCenterlinePoses[4].getTranslation(), Rotation2d.fromDegrees(-180)))
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2, new Rotation2d()))
                            .getTranslation()))
                .build()));

    paths.put(
        "N6-S21-C432_driveToC3",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(0, 2, new Rotation2d()))
                            .getTranslation()))
                .addPoseWaypoint(intakingCenterlinePoses[3])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(-1, 2, new Rotation2d()))
                            .getTranslation()))
                .build()));

    paths.put(
        "N6-S21-C432_driveToC2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(-1, 2, new Rotation2d()))
                            .getTranslation()))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.Stage.center.getTranslation(), new Rotation2d(Math.PI)))
                .addPoseWaypoint(intakingCenterlinePoses[2])
                .addPoseWaypoint(
                    getShootingPose(
                        FieldConstants.Stage.center
                            .transformBy(new Transform2d(-.5, .6, new Rotation2d(Math.PI)))
                            .getTranslation()))
                .build()));

    // Unethical Auto

    paths.put(
        "unethicalAuto_driveToC4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceFace)
                .addTranslationWaypoint(
                    intakingCenterlinePoses[4]
                        .transformBy(new Transform2d(1.4, .15, new Rotation2d()))
                        .getTranslation())
                .addPoseWaypoint(
                    new Pose2d(
                        intakingCenterlinePoses[4].getTranslation(), Rotation2d.fromDegrees(200)))
                .build()));

    paths.put(
        "unethicalAuto_driveToC0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getLastWaypointPose("unethicalAuto_driveToC4"))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[3],
                        Rotation2d.fromDegrees(120)))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[0],
                        Rotation2d.fromDegrees(120)))
                .build()));

    paths.put(
        "unethicalAuto_driveToPodium",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getLastWaypointPose("unethicalAuto_driveToC0"))
                .addTranslationWaypoint(
                    FieldConstants.Stage.podiumLeg
                        .transformBy(new Transform2d(-1, 0, new Rotation2d()))
                        .getTranslation())
                .addPoseWaypoint(
                    getShootingPose(
                        new Pose2d(
                                FieldConstants.StagingLocations.spikeTranslations[0],
                                new Rotation2d(0))
                            .transformBy(new Transform2d(-doNotHitOffset, .1, new Rotation2d()))
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

  public static Pose2d getLastWaypointPose(String trajectoryName) {
    List<PathSegment> trajectory = paths.get(trajectoryName);
    Waypoint waypoint =
        trajectory
            .get(trajectory.size() - 1)
            .getWaypoints(trajectory.get(trajectory.size() - 1).getWaypointsCount() - 1);
    return new Pose2d(
        waypoint.getX(), waypoint.getY(), Rotation2d.fromRadians(waypoint.getHeadingConstraint()));
  }

  private DriveTrajectories() {}
}
