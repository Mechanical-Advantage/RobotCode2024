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
import edu.wpi.first.math.util.Units;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class DriveTrajectories {
  public static final Map<String, List<PathSegment>> paths = new HashMap<>();

  static {
    paths.put(
        "driveStraight",
        List.of(
            PathSegment.newBuilder()
                .addPoseWaypoint(
                    new Pose2d(
                        Units.inchesToMeters(60.112),
                        Units.inchesToMeters(161.638),
                        new Rotation2d()))
                .addPoseWaypoint(
                    new Pose2d(
                        Units.inchesToMeters(130), Units.inchesToMeters(70), new Rotation2d()))
                .addPoseWaypoint(
                    new Pose2d(
                        Units.inchesToMeters(318.640),
                        Units.inchesToMeters(29.003),
                        new Rotation2d()))
                .addPoseWaypoint(
                    new Pose2d(
                        Units.inchesToMeters(130), Units.inchesToMeters(70), new Rotation2d()))
                .build()));
  }
}
