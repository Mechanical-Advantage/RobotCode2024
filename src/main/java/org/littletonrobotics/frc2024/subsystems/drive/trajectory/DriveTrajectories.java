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
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.PathSegment;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleVelocityConstraint;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.Waypoint;

@ExtensionMethod({TrajectoryGenerationHelpers.class, GeomUtil.class})
public class DriveTrajectories {
  public static final Map<String, List<PathSegment>> paths = new HashMap<>();
  public static final List<Function<Set<String>, Map<String, List<PathSegment>>>> suppliedPaths =
      new ArrayList<>(); // List of functions that take a set of completed paths and return a map of
  // trajectories to generate (or null if they cannot be generated yet)

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
  public static final Pose2d startingAmpEdge =
      new Pose2d(startingLineX - 0.5, Amp.ampBottomY - 0.45, Rotation2d.fromDegrees(180.0));
  public static final Pose2d startingFarSource =
      new Pose2d(FieldConstants.startingLineX - 0.5, 1.57, Rotation2d.fromDegrees(180));
  // Subwoofer starting locations
  public static final Pose2d startingSourceSubwoofer =
      FieldConstants.Subwoofer.sourceFaceCorner.transformBy(
          GeomUtil.toTransform2d(-Units.inchesToMeters(17.0), Units.inchesToMeters(17.0)));
  public static final Pose2d startingAmpSubwoofer =
      FieldConstants.Subwoofer.ampFaceCorner.transformBy(
          GeomUtil.toTransform2d(-Units.inchesToMeters(17.0), -Units.inchesToMeters(17.0)));

  // Shooting poses
  public static final Pose2d stageLeftShootingPose =
      getShootingPose(
          FieldConstants.Stage.ampLeg.getTranslation().plus(new Translation2d(-0.7, 1.15)));
  public static final Pose2d stageRightShootingPose =
      getShootingPose(
          FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(0.5, -1.3)));
  public static final Pose2d stageRightCloseShootingPose =
      getShootingPose(
          FieldConstants.Stage.podiumLeg.getTranslation().plus(new Translation2d(-0.5, -1.8)));
  public static final Pose2d stageCenterShootingPose =
      getShootingPose(
          FieldConstants.Stage.podiumLeg
              .getTranslation()
              .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.5)
              .plus(new Translation2d(-0.4, 0.2)));
  public static final Pose2d CA_lastCenterlineShot =
      getShootingPose(
          StagingLocations.spikeTranslations[2]
              .interpolate(StagingLocations.spikeTranslations[1], 0.3)
              .plus(new Translation2d(-0.25, 0.0)));

  // Avoidance points
  public static final Translation2d stageLeftAvoidance =
      new Translation2d(
          FieldConstants.wingX,
          MathUtil.interpolate(FieldConstants.Stage.ampLeg.getY(), FieldConstants.fieldWidth, 0.4));
  public static final Translation2d stageRightAvoidance =
      new Translation2d(
          FieldConstants.wingX,
          MathUtil.interpolate(FieldConstants.Stage.sourceLeg.getY(), 0.0, 0.4));
  public static final Translation2d stageCenterAvoidance =
      FieldConstants.Stage.sourceLeg
          .getTranslation()
          .interpolate(FieldConstants.Stage.ampLeg.getTranslation(), 0.62);

  // Drive straight path
  // (Used for preload of trajectory classes in drive constructor)
  static {
    paths.put(
        "driveStraight",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(new Pose2d())
                .addPoseWaypoint(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(180.0)))
                .build()));
  }

  // Thinking-on-your-feet
  private static final Waypoint thinkingStartWaypoint =
      Waypoint.newBuilder()
          .fromPose(
              new Pose2d(
                  FieldConstants.wingX + 1.0,
                  FieldConstants.StagingLocations.centerlineTranslations[4].getY(),
                  Rotation2d.fromDegrees(180.0)))
          .setVehicleVelocity(
              VehicleVelocityConstraint.newBuilder().setVx(3.0).setVy(0.0).setOmega(0.0).build())
          .build();

  static {
    final double centerlineIntakeOffset = 0.4;
    final double intakeVelocity = 2.2;

    // First intake
    paths.put(
        "thinking_firstIntake",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(thinkingStartWaypoint)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(180.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, Units.inchesToMeters(8))
                                .toTransform2d()))
                .build(),
            PathSegment.newBuilder()
                .setMaxVelocity(intakeVelocity)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(100.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .build(),
            PathSegment.newBuilder()
                .setMaxVelocity(intakeVelocity)
                .setMaxOmega(0.0)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(100.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(100.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[0],
                        Rotation2d.fromDegrees(100.0)))
                .build()));

    // Second intake
    paths.put(
        "thinking_secondIntake",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageLeftShootingPose)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .build(),
            PathSegment.newBuilder()
                .setMaxVelocity(intakeVelocity)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(100.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(100.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[0],
                        Rotation2d.fromDegrees(100.0)))
                .build()));

    // Far intake
    paths.put(
        "thinking_farIntake",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(stageCenterShootingPose)
                .addTranslationWaypoint(stageCenterAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            Rotation2d.fromDegrees(135.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .build(),
            PathSegment.newBuilder()
                .setMaxVelocity(intakeVelocity)
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[1],
                            Rotation2d.fromDegrees(100.0))
                        .transformBy(
                            new Translation2d(centerlineIntakeOffset, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.StagingLocations.centerlineTranslations[0],
                        Rotation2d.fromDegrees(100.0)))
                .build()));

    // Return trajectories
    for (String intakeName :
        List.of("thinking_firstIntake", "thinking_secondIntake", "thinking_farIntake")) {
      suppliedPaths.add(
          (completedPaths) -> {
            if (!completedPaths.contains(intakeName)) return null;

            final double minDistance = 0.15;

            Map<String, List<PathSegment>> returnPaths = new HashMap<>();
            var intakeTrajectory = new HolonomicTrajectory(intakeName);
            int index = -1;
            int validIndex = -1;
            Translation2d lastTranslation = new Translation2d();
            for (var state : intakeTrajectory.getStates()) {
              index++;
              Translation2d translation = state.getPose().getTranslation();
              if ((translation.getDistance(lastTranslation) > minDistance
                      && translation.getX() > FieldConstants.wingX + 1.0)
                  || index == intakeTrajectory.getStates().length - 1) {
                validIndex++;
                lastTranslation = translation;

                boolean returnToStage = translation.getY() < FieldConstants.Stage.ampLeg.getY();
                paths.put(
                    intakeName + "Return" + String.format("%03d", validIndex),
                    List.of(
                        PathSegment.newBuilder()
                            .addContinuationWaypoint(state)
                            .addTranslationWaypoint(
                                returnToStage ? stageCenterAvoidance : stageLeftAvoidance, 15)
                            .addPoseWaypoint(
                                returnToStage ? stageCenterShootingPose : stageLeftShootingPose, 15)
                            .build()));
                if (!intakeName.equals("thinking_firstIntake")) {
                  paths.put(
                      intakeName + "CAReturn" + String.format("%03d", validIndex),
                      List.of(
                          PathSegment.newBuilder()
                              .addContinuationWaypoint(state)
                              .addTranslationWaypoint(stageLeftAvoidance, 20)
                              .addPoseWaypoint(CA_lastCenterlineShot)
                              .build()));
                }
              }
            }
            return returnPaths;
          });
    }
  }

  // Davis Spiky Auto (named "spiky_XXX")
  static {
    final double shootingVelocity = 0.7;
    final double spikeIntakeOffset = 0.55;
    final double spikePrepareIntakeOffset = 0.3; // Added ontop of spikeIntakeOffset

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
                        .plus(betweenSpikeShotTranslation)),
                15)
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
                    spikeShootingPoses[1].transformBy(new Translation2d(0.15, 0.0).toTransform2d()))
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
                    spikeShootingPoses[1].transformBy(new Translation2d(0.15, 0.0).toTransform2d()))
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(spikeShootingPoses[1])
                .setStraightLine(true)
                .build()));

    // Start thinking-on-your-feet
    for (int spikeIndex = 0; spikeIndex < 3; spikeIndex++) {
      Pose2d spikePose = spikeShootingPoses[spikeIndex];

      // Start segment at spike pose
      var spikeToCenterline = PathSegment.newBuilder().addPoseWaypoint(spikePose).build();

      // Avoid podium leg if spike 0
      if (spikeIndex == 0) {
        spikeToCenterline =
            spikeToCenterline.toBuilder().addTranslationWaypoint(podiumAvoidance).build();
      }

      // End at intake waypoint
      spikeToCenterline = spikeToCenterline.toBuilder().addWaypoints(thinkingStartWaypoint).build();

      // Add path
      paths.put("spiky_spike" + spikeIndex + "ToFirstIntake", List.of(spikeToCenterline));
    }
  }

  // Davis CA Auto (named "CA_XXX")
  static {
    final double shootingVelocity = 0.7;

    paths.put(
        ("CA_startToCenterline"),
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmp)
                .addPoseWaypoint(
                    getShootingPose(StagingLocations.spikeTranslations[2])
                        .transformBy(GeomUtil.toTransform2d(0.4, 0.0)))
                .setMaxVelocity(1.5)
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(StagingLocations.spikeTranslations[2])
                        .transformBy(GeomUtil.toTransform2d(0.1, 0.0)))
                .setMaxVelocity(shootingVelocity)
                .build(),
            PathSegment.newBuilder().addWaypoints(thinkingStartWaypoint).build()));

    paths.put(
        ("CA_centerlineToSpikes"),
        List.of(
            // Shoot last centerline while moving in front of spike 1
            PathSegment.newBuilder()
                .addPoseWaypoint(CA_lastCenterlineShot)
                .addPoseWaypoint(
                    getShootingPose(
                        StagingLocations.spikeTranslations[2]
                            .interpolate(StagingLocations.spikeTranslations[1], 0.4)
                            .plus(new Translation2d(-0.65, 0.0))))
                .build(),

            // Intake spike 1
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], Rotation2d.fromDegrees(160.0))
                        .transformBy(GeomUtil.toTransform2d(0.8, 0.0)))
                .addPoseWaypoint(
                    new Pose2d(StagingLocations.spikeTranslations[1], Rotation2d.fromDegrees(160.0))
                        .transformBy(GeomUtil.toTransform2d(0.5, 0.0)))
                .build(),

            // Shoot spike 1
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(
                        StagingLocations.spikeTranslations[1]
                            .interpolate(StagingLocations.spikeTranslations[0], 0.4)
                            .plus(new Translation2d(-0.65, 0.0))))
                .setMaxVelocity(1.2)
                .build(),

            // Intake and shoot spike 0
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    getShootingPose(StagingLocations.spikeTranslations[0])
                        .transformBy(GeomUtil.toTransform2d(0.5, 0.0)))
                .build()));
  }

  // Davis Speedy Auto (named "speedy_XXX")
  static {
    final Pose2d stageLeftShootingPose =
        getShootingPose(
            FieldConstants.Stage.ampLeg.getTranslation().plus(new Translation2d(-0.7, 0.9)));
    final Translation2d stageLeftAvoidance =
        new Translation2d(
            FieldConstants.wingX,
            MathUtil.interpolate(
                FieldConstants.Stage.ampLeg.getY(), FieldConstants.fieldWidth, 0.3));

    paths.put(
        "speedy_ampToCenterline4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmpEdge)
                .addPoseWaypoint(
                    new Pose2d(
                        new Translation2d(
                            StagingLocations.spikeTranslations[2].getX() - 0.2,
                            MathUtil.interpolate(
                                StagingLocations.spikeTranslations[2].getY(), fieldWidth, 0.53)),
                        Rotation2d.fromDegrees(180)))
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(
                        new Translation2d(
                            StagingLocations.spikeTranslations[2].getX() + 0.3,
                            MathUtil.interpolate(
                                StagingLocations.spikeTranslations[2].getY(), fieldWidth, 0.53)),
                        Rotation2d.fromDegrees(180)))
                .addPoseWaypoint(
                    new Pose2d(
                        new Translation2d(
                            Stage.ampLeg.getX() - 0.75,
                            MathUtil.interpolate(
                                StagingLocations.spikeTranslations[2].getY(), fieldWidth, 0.53)),
                        Rotation2d.fromDegrees(180)))
                .setStraightLine(true)
                .setMaxOmega(0)
                .build(),
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-170.0))
                        .transformBy(new Translation2d(0.6, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-170.0))
                        .transformBy(new Translation2d(0.3, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[4],
                            Rotation2d.fromDegrees(-170.0))
                        .transformBy(new Translation2d(0.6, 0.0).toTransform2d()))
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
                            Rotation2d.fromDegrees(150.0))
                        .transformBy(new Translation2d(0.75, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(150.0))
                        .transformBy(new Translation2d(0.25, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[3],
                            Rotation2d.fromDegrees(150.0))
                        .transformBy(new Translation2d(0.75, 0.0).toTransform2d()))
                .addPoseWaypoint(stageLeftShootingPose)
                .build()));
    paths.put(
        "speedy_centerline3ToCenterline2",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("speedy_centerline4ToCenterline3"))
                .addTranslationWaypoint(stageLeftAvoidance)
                .addPoseWaypoint(
                    new Pose2d(
                            StagingLocations.centerlineTranslations[2],
                            stageLeftAvoidance
                                .minus(StagingLocations.centerlineTranslations[2])
                                .getAngle())
                        .transformBy(new Translation2d(1, 0.0).toTransform2d()))
                .addPoseWaypoint(
                    new Pose2d(
                            FieldConstants.StagingLocations.centerlineTranslations[2],
                            stageLeftAvoidance
                                .minus(StagingLocations.centerlineTranslations[2])
                                .getAngle())
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
                        .transformBy(new Translation2d(0.25, -0.08).toTransform2d()))
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
            PathSegment.newBuilder()
                .addTranslationWaypoint(stageRightAvoidance)
                .addPoseWaypoint(stageRightShootingPose)
                .build()));
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
            PathSegment.newBuilder().addTranslationWaypoint(stageRightAvoidance).build(),
            intakeCenterline1,
            PathSegment.newBuilder()
                .addTranslationWaypoint(stageRightAvoidance)
                .addPoseWaypoint(stageRightShootingPose)
                .build()));
    paths.put(
        "unethical_centerline0ToCenterline1",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("unethical_grabCenterline0"))
                .addTranslationWaypoint(stageRightAvoidance)
                .build(),
            intakeCenterline1,
            PathSegment.newBuilder()
                .addTranslationWaypoint(stageRightAvoidance)
                .addPoseWaypoint(stageRightCloseShootingPose)
                .build()));
    paths.put(
        "unethical_centerline1ToCenterline0",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("unethical_grabCenterline1"))
                .build(),
            intakeCenterline0,
            PathSegment.newBuilder().addPoseWaypoint(stageRightCloseShootingPose).build()));
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
                        new Transform2d(0.65, 0.0, Rotation2d.fromDegrees(180.0))))
                .addPoseWaypoint(stageRightCloseShootingPose)
                .build()));
    paths.put(
        "unethical_driveToSource",
        List.of(
            PathSegment.newBuilder()
                .addWaypoints(getLastWaypoint("unethical_grabEjected"))
                .addPoseWaypoint(new Pose2d(FieldConstants.wingX, 1.0, Rotation2d.fromDegrees(180)))
                .build(),
            PathSegment.newBuilder()
                .addTranslationWaypoint(new Translation2d((FieldConstants.fieldLength) - 2, 1.2))
                .setStraightLine(true)
                .setMaxOmega(0)
                .build()));
  }

  // Davis Inspirational Auto (named "inspirational_XXX")
  static {
    paths.put(
        "inspirational_leaveFromSource",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceSubwoofer)
                .addTranslationWaypoint(
                    FieldConstants.Stage.podiumLeg
                        .getTranslation()
                        .plus(new Translation2d(0.0, -2.0)))
                .build()));
    paths.put(
        "inspirational_leaveFromCenter",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingCenter)
                .addTranslationWaypoint(
                    FieldConstants.StagingLocations.spikeTranslations[1].plus(
                        new Translation2d(1.0, 0.0)))
                .build()));
    paths.put(
        "inspirational_leaveFromAmp",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingAmpSubwoofer)
                .addTranslationWaypoint(
                    FieldConstants.StagingLocations.spikeTranslations[2].plus(
                        new Translation2d(1.0, 0.0)))
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
