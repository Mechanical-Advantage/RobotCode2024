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
          new Transform2d(DriveConstants.driveConfig.bumperWidthX() / 2, 0, new Rotation2d(0)));

  // Center intake locations
  private static final double intakeOffset = 0.5;
  private static final Pose2d[] intakingPoses = new Pose2d[5];

  static {
    // Find locations for intaking centerline gamepieces
    for (int i = 0; i < 5; i++) {
      Translation2d centerLineTranslation =
          FieldConstants.StagingLocations.centerlineTranslations[i];
      intakingPoses[i] =
          new Pose2d(
              centerLineTranslation.minus(new Translation2d(intakeOffset, 0)),
              new Rotation2d(Math.PI));
    }

    // Add paths
    paths.put(
        "driveToCenterline4",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(startingSourceFace)
                .addTranslationWaypoint(new Translation2d(3.5, 2.5))
                .addPoseWaypoint(intakingPoses[0])
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .build()));

    paths.put(
        "driveToCenterline3",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .addTranslationWaypoint(new Translation2d(5.5, 1.4))
                .addPoseWaypoint(intakingPoses[1])
                .addTranslationWaypoint(new Translation2d(5.5, 1.4))
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .build()));
    paths.put(
        "driveToPodium",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(getShootingPose(new Translation2d(3.5, 2.5)))
                .addTranslationWaypoint(new Translation2d(1.6 - Units.inchesToMeters(20), 3.75))
                .addPoseWaypoint(
                    new Pose2d(
                        FieldConstants.StagingLocations.spikeTranslations[0].getX()
                            - Units.inchesToMeters(20),
                        FieldConstants.StagingLocations.spikeTranslations[0].getY(),
                        new Rotation2d(Math.PI)))
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
