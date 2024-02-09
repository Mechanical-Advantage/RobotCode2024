package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class DriveTrajectories {
    public static final List<PathSegment> driveStraight =
            List.of(
                    PathSegment.newBuilder()
                            .addPoseWaypoint(new Pose2d())
                            .addPoseWaypoint(new Pose2d(2.0, 0.0, new Rotation2d()), 100)
                            .build());
}
