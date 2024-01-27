package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public class AprilTagVisionConstants {
    public static final double ambiguityThreshold = 0.15;
    public static final double targetLogTimeSecs = 0.1;
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.75;
    public static final double xyStdDevCoefficient = 0.01;
    public static final double thetaStdDevCoefficient = 0.01;

    public static final Pose3d[] cameraPoses =
            switch (Constants.getRobot()) {
                case RAINBOWT -> new Pose3d[] {
                    new Pose3d(), // TODO: what are these again?
                    new Pose3d()
                };
                default -> new Pose3d[] {};
            };

    public static final String[] cameraNames =
            switch (Constants.getRobot()) {
                case RAINBOWT -> new String[] {
                    "northstar_0",
                    "northstar_1"
                };
                default -> new String[] {};
            };
}
