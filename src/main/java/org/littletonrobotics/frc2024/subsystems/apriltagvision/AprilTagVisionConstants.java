package org.littletonrobotics.frc2024.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2024.Constants;

public class AprilTagVisionConstants {
    public static final double ambiguityThreshold = 0.15;
    public static final double targetLogTimeSecs = 0.1;
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.75;
    public static final double xyStdDevCoefficient = 0.01;
    public static final double thetaStdDevCoefficient = 0.01;

    public static final Pose3d[] cameraPoses =
            switch (Constants.getRobot()) {
                case DEVBOT ->
                        new Pose3d[] {
                            new Pose3d(
                                            Units.inchesToMeters(9.735),
                                            Units.inchesToMeters(9.974),
                                            Units.inchesToMeters(6.839),
                                            new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0))
                                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))
                        };
                default -> new Pose3d[] {};
            };

    public static final String[] cameraNames =
            switch (Constants.getRobot()) {
                case DEVBOT -> new String[] {"northstar_0"};
                default -> new String[] {};
            };
}
