package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
                .addPoseWaypoint(new Pose2d())
                .addPoseWaypoint(new Pose2d(2.0, 0.0, new Rotation2d()), 100)
                .addPoseWaypoint(new Pose2d(4.0, 2.0, new Rotation2d(Math.PI)))
                .build()));
  }
}
