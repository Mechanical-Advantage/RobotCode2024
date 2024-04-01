// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.frc2024.FieldConstants.*;

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
  public static final Pose2d startingFarSource =
      new Pose2d(FieldConstants.startingLineX - 0.5, 1.57, Rotation2d.fromDegrees(180));

  // Shooting poses
  public static final Pose2d stageLeftShootingPose =
      getShootingPose(
          FieldConstants.Stage.ampLeg.getTranslation().plus(new Translation2d(-0.7, 1.15)));
  public static final Pose2d stageRightShootingPose =
      getShootingPose(
          FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(0.5, -1.3)));
  public static final Pose2d stageCenterShootingPose =
      getShootingPose(
          FieldConstants.Stage.podiumLeg
              .getTranslation()
              .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.5)
              .plus(new Translation2d(-0.4, 0.2)));

  // Avoidance points
  public static final Translation2d stageLeftAvoidance =
      new Translation2d(
          FieldConstants.wingX,
          MathUtil.interpolate(FieldConstants.Stage.ampLeg.getY(), FieldConstants.fieldWidth, 0.4));
  public static final Translation2d stageRightAvoidance =
      FieldConstants.Stage.sourceLeg.getTranslation().plus(new Translation2d(0.0, -1.2));
  public static final Translation2d stageCenterAvoidance =
      FieldConstants.Stage.sourceLeg
          .getTranslation()
          .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.62);

  // Davis Spiky Auto (named "spiky_XXX")
  static {
    final double shootingVelocity = 0.7;
    final double spikeIntakeOffset = 0.55;
    final double spikePrepareIntakeOffset = 0.3; // Added ontop of spikeIntakeOffset
    final double centerlineIntakeOffset = 0.25;
    final double centerlinePrepareIntakeOffset = 1.0;

    final Rotation2d spike0To1IntakeRotation = Rotation2d.fromDegrees(-160.0);
    final Rotation2d spike2To1IntakeRotation =
        new Rotation2d(spike0To1IntakeRotation.getCos(), -spike0To1IntakeRotation.getSin());

    Translation2d podiumAvoidance =
        StagingLocations.spikeTranslations[0].plus(new Translation2d(-0.5, 0.9));

    Pose2d[] spikeShootingPoses = new Pose2d[3];
    for (int i = 0; i < 3; i++) {
      spikeShootingPoses[i] =
          getShootingPose(StagingLocations.spikeTranslations[i])
              .transformBy(new Translation2d(spikeIntakeOffset, 0.0).toTransform2d());
    }

    // All starting locations to first spikes segments
    var sourceStartToSpike0 =
        PathSegment.newBuilder()
            .addPoseWaypoint(getShootingPose(startingSource.getTranslation()))
            .addPoseWaypoint(
                spikeShootingPoses[0].transformBy(
                    new Translation2d(spikePrepareIntakeOffset, 0.0).toTransform2d()))
            .addPoseWaypoint(spikeShootingPoses[0])
            .build();
    var centerStartToSpike0 =
        PathSegment.newBuilder()
            .addPoseWaypoint(getShootingPose(startingCenter.getTranslation()))
            .addPoseWaypoint(
                spikeShootingPoses[0].transformBy(
                    new Translation2d(spikePrepareIntakeOffset, 0.0).toTransform2d()))
            .addPoseWaypoint(spikeShootingPoses[0])
            .build();
    var centerStartToSpike1 =
        PathSegment.newBuilder()
            .addPoseWaypoint(getShootingPose(startingCenter.getTranslation()))
            .addPoseWaypoint(spikeShootingPoses[1])
            .build();
    var ampStartToSpike2 =
        PathSegment.newBuilder()
            .addPoseWaypoint(getShootingPose(startingAmp.getTranslation()))
            .addPoseWaypoint(
                spikeShootingPoses[2].transformBy(
                    new Translation2d(spikePrepareIntakeOffset, 0.0).toTransform2d()))
            .addPoseWaypoint(spikeShootingPoses[2])
            .build();

    // Between spike shooting segments
    final double betweenSpikeShotS = 0.4;
    final Translation2d betweenSpikeShotTranslation = new Translation2d(-0.65, 0.0);
    var spike0ToSpike1Shot =
        PathSegment.newBuilder()
            .addPoseWaypoint(
                getShootingPose(
                    StagingLocations.spikeTranslations[0]
                        .interpolate(StagingLocations.spikeTranslations[1], betweenSpikeShotS)
                        .plus(betweenSpikeShotTranslation)))
            .setMaxVelocity(shootingVelocity)
            .build();
    var spike1ToSpike2Shot =
        PathSegment.newBuilder()
            .addPoseWaypoint(
                getShootingPose(
                    StagingLocations.spikeTranslations[1]
                        .interpolate(StagingLocations.spikeTranslations[2], betweenSpikeShotS)
                        .plus(betweenSpikeShotTranslation)))
            .setMaxVelocity(shootingVelocity)
            .build();
    var spike2ToSpike1Shot =
        PathSegment.newBuilder()
            .addPoseWaypoint(
                getShootingPose(
                    StagingLocations.spikeTranslations[2]
                        .interpolate(StagingLocations.spikeTranslations[1], betweenSpikeShotS)
                        .plus(betweenSpikeShotTranslation)))
            .setMaxVelocity(shootingVelocity)
            .build();
    var spike1ToSpike0Shot =
        PathSegment.newBuilder()
            .addPoseWaypoint(
                getShootingPose(
                    StagingLocations.spikeTranslations[1]
                        .interpolate(StagingLocations.spikeTranslations[0], betweenSpikeShotS)
                        .plus(betweenSpikeShotTranslation)))
            .setMaxVelocity(shootingVelocity)
            .build();

    // Trajectories for 3 and 2 spikes
    paths.put(
        "spiky_sourceStart3",
        List.of(
            sourceStartToSpike0,
            spike0ToSpike1Shot,
            // Intake spike 1
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], spike0To1IntakeRotation)
                        .transformBy(
                            new Translation2d(spikeIntakeOffset + spikePrepareIntakeOffset, 0.0)
                                .toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], spike0To1IntakeRotation)
                        .transformBy(new Translation2d(spikeIntakeOffset, 0.0).toTransform2d()))
                .build(),
            spike1ToSpike2Shot,
            // Intake and shoot spike 2
            PathSegment.newBuilder().addPoseWaypoint(spikeShootingPoses[2]).build()));
    paths.put(
        "spiky_sourceStart2",
        List.of(
            sourceStartToSpike0,
            spike0ToSpike1Shot,
            // Intake and shoot spike 1
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    spikeShootingPoses[1].transformBy(
                        new Translation2d(spikePrepareIntakeOffset, 0.0).toTransform2d()))
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(spikeShootingPoses[1])
                .setStraightLine(true)
                .build()));
    paths.put(
        "spiky_centerStart3",
        List.of(
            centerStartToSpike0,
            spike0ToSpike1Shot,
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], spike0To1IntakeRotation)
                        .transformBy(
                            new Translation2d(spikeIntakeOffset + spikePrepareIntakeOffset, 0.0)
                                .toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], spike0To1IntakeRotation)
                        .transformBy(new Translation2d(spikeIntakeOffset, 0.0).toTransform2d()))
                .build(),
            spike1ToSpike2Shot,
            PathSegment.newBuilder().addPoseWaypoint(spikeShootingPoses[2]).build()));
    paths.put(
        "spiky_centerStart2",
        List.of(
            centerStartToSpike1,
            spike1ToSpike2Shot,
            PathSegment.newBuilder().addPoseWaypoint(spikeShootingPoses[2]).build()));
    paths.put(
        "spiky_ampStart3",
        List.of(
            ampStartToSpike2,
            spike2ToSpike1Shot,
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], spike2To1IntakeRotation)
                        .transformBy(
                            new Translation2d(spikeIntakeOffset + spikePrepareIntakeOffset, 0.0)
                                .toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], spike2To1IntakeRotation)
                        .transformBy(new Translation2d(spikeIntakeOffset, 0.0).toTransform2d()))
                .build(),
            spike1ToSpike0Shot,
            PathSegment.newBuilder().addPoseWaypoint(spikeShootingPoses[0]).build()));
    paths.put(
        "spiky_ampStart2",
        List.of(
            ampStartToSpike2,
            spike2ToSpike1Shot,
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    spikeShootingPoses[1].transformBy(
                        new Translation2d(spikePrepareIntakeOffset, 0.0).toTransform2d()))
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(spikeShootingPoses[1])
                .setStraightLine(true)
                .build()));

    // Generate trajectories for centerline shots
    PathSegment[][] spikeToCenterlineIntakeSegments = new PathSegment[3][3];
    PathSegment[] shotToCenterlineIntakeSegments = new PathSegment[3];
    PathSegment[] centerlineToShotSegments = new PathSegment[3];

    for (int i = 0; i < 3; i++) {
      int centerlineIndex = 4 - i;
      // Centerline intake to shot
      Translation2d centerlineNote = StagingLocations.centerlineTranslations[centerlineIndex];
      if (centerlineIndex != 4) {
        centerlineToShotSegments[i] =
            PathSegment.newBuilder()
                .addTranslationWaypoint(stageLeftAvoidance)
                .addPoseWaypoint(stageLeftShootingPose)
                .build();
      } else {
        centerlineToShotSegments[i] =
            PathSegment.newBuilder().addPoseWaypoint(stageLeftShootingPose).build();
      }

      // Make shot to centerline intake segments
      PathSegment shotToCenterlineIntake;
      Rotation2d shotToCenterlineIntakeOrientation =
          (centerlineIndex == 4)
              ? stageLeftShootingPose.getTranslation().minus(centerlineNote).getAngle()
              : stageLeftAvoidance.minus(centerlineNote).getAngle();
      shotToCenterlineIntake =
          PathSegment.newBuilder().addPoseWaypoint(stageLeftShootingPose).build();
      if (centerlineIndex != 4) {
        shotToCenterlineIntake =
            shotToCenterlineIntake.toBuilder().addTranslationWaypoint(stageLeftAvoidance).build();
      }
      shotToCenterlineIntake =
          shotToCenterlineIntake.toBuilder()
              .addPoseWaypoint(
                  new Pose2d(centerlineNote, shotToCenterlineIntakeOrientation)
                      .transformBy(
                          new Translation2d(
                                  centerlineIntakeOffset + centerlinePrepareIntakeOffset, 0.0)
                              .toTransform2d()))
              .addPoseWaypoint(
                  new Pose2d(centerlineNote, shotToCenterlineIntakeOrientation)
                      .transformBy(new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
              .addPoseWaypoint(
                  new Pose2d(centerlineNote, shotToCenterlineIntakeOrientation)
                      .transformBy(
                          new Translation2d(
                                  centerlineIntakeOffset + centerlinePrepareIntakeOffset, 0.0)
                              .toTransform2d()))
              .build();
      shotToCenterlineIntakeSegments[i] = shotToCenterlineIntake;

      // Segments for all spikes to centerline intake
      for (int spikeIndex = 0; spikeIndex < 3; spikeIndex++) {
        Pose2d spikePose = spikeShootingPoses[spikeIndex];
        Rotation2d spikeTocenterlineIntakeOrientation =
            spikePose.getTranslation().minus(centerlineNote).getAngle();
        // Start segment at spike pose
        var spikeToCenterline = PathSegment.newBuilder().addPoseWaypoint(spikePose).build();
        // Avoid podium leg if spike 0
        if (spikeIndex == 0) {
          spikeToCenterline =
              spikeToCenterline.toBuilder().addTranslationWaypoint(podiumAvoidance).build();
          spikeTocenterlineIntakeOrientation = podiumAvoidance.minus(centerlineNote).getAngle();
        }
        // Use wing left avoidance if not centerline 4
        if (centerlineIndex != 4) {
          spikeToCenterline =
              spikeToCenterline.toBuilder().addTranslationWaypoint(stageLeftAvoidance).build();
          spikeTocenterlineIntakeOrientation = stageLeftAvoidance.minus(centerlineNote).getAngle();
        }
        // Add intake to segment
        spikeToCenterline =
            spikeToCenterline.toBuilder()
                .addPoseWaypoint(
                    new Pose2d(centerlineNote, spikeTocenterlineIntakeOrientation)
                        .transformBy(
                            new Translation2d(
                                    centerlineIntakeOffset + centerlinePrepareIntakeOffset, 0.0)
                                .toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(centerlineNote, spikeTocenterlineIntakeOrientation)
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(centerlineNote, spikeTocenterlineIntakeOrientation)
                        .transformBy(
                            new Translation2d(
                                    centerlineIntakeOffset + centerlinePrepareIntakeOffset, 0.0)
                                .toTransform2d()))
                .build();
        spikeToCenterlineIntakeSegments[spikeIndex][i] = spikeToCenterline;
      }
    }

    // Generate trajectories
    for (int i = 0; i < 3; i++) {
      int centerlineIndex = 4 - i;
      paths.put(
          "spiky_shotToCenterline" + centerlineIndex,
          List.of(shotToCenterlineIntakeSegments[i], centerlineToShotSegments[i]));
      for (int spikeIndex = 0; spikeIndex < 3; spikeIndex++) {
        paths.put(
            "spiky_spike" + spikeIndex + "ToCenterline" + centerlineIndex,
            List.of(spikeToCenterlineIntakeSegments[spikeIndex][i], centerlineToShotSegments[i]));
      }
    }
  }

  // Davis Speedy Auto (named "speedy_XXX")
  static {
  }

  // Davis Ethical Auto (named "ethical_XXX")
  static {
    Translation2d podiumRightAvoidance =
        Stage.podiumLeg.getTranslation().plus(new Translation2d(0.5, -2.0));

    paths.put(
        "ethical_grabCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(startingSource.getTranslation()))
                .addTranslationWaypoint(podiumRightAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[0],
                            Rotation2d.fromDegrees(170.0))
                        .transformBy(new Translation2d(0.25, -0.07).toTransform2d()))
                .addTranslationWaypoint(stageRightAvoidance)
                .addPoseWaypoint(stageRightShootingPose)
                .build()));
    paths.put(
        "ethical_grabCenterline1",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageRightShootingPose)
                .addTranslationWaypoint(stageRightAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(-160.0))
                        .transformBy(new Translation2d(0.15, -0.05).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(160.0))
                        .transformBy(new Translation2d(0.15, -0.05).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageCenterShootingPose)
                .build()));
    paths.put(
        "ethical_grabCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageCenterShootingPose)
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(180.0))
                        .transformBy(new Translation2d(0.3, -0.05).toTransform2d()))
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(stageCenterShootingPose)
                .build()));
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
                    new Pose2d(FieldConstants.wingX + 0.75, 1.25, Rotation2d.fromDegrees(180)))
                .build(),
            PathSegment.newBuilder()
                .addTranslationWaypoint(new Translation2d((FieldConstants.fieldLength) - 2, 1.25))
                .setStraightLine(true)
                .setMaxOmega(0)
                .build()));
  }

  /** Calculates aimed pose from translation. */
  private static Pose2d getShootingPose(Translation2d translation) {
    return new Pose2d(
        translation, Speaker.centerSpeakerOpening.toTranslation2d().minus(translation).getAngle());
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
