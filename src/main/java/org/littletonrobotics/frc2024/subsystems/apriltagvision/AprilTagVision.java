package org.littletonrobotics.frc2024.subsystems.apriltagvision;

import static org.littletonrobotics.frc2024.RobotState.VisionObservation;
import static org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionConstants.*;
import static org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem for AprilTag vision. */
@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends VirtualSubsystem {

    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputs[] inputs;

    private final RobotState robotState = RobotState.getInstance();
    private boolean enableVisionUpdates = true;
    private final Alert enableVisionUpdatesAlert =
            new Alert("Vision updates are temporarily disabled.", Alert.AlertType.WARNING);
    private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    public AprilTagVision(AprilTagVisionIO... io) {
        System.out.println("[Init] Creating AprilTagVision");
        this.io = io;
        inputs = new AprilTagVisionIOInputs[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new AprilTagVisionIOInputs();
        }

        // Create map of last frame times for instances
        for (int i = 0; i < io.length; i++) {
            lastFrameTimes.put(i, 0.0);
        }

        // Create map of last detection times for tags
        FieldConstants.aprilTags
                .getTags()
                .forEach(
                        (AprilTag tag) -> {
                            lastTagDetectionTimes.put(tag.ID, 0.0);
                        });
    }

    /** Sets whether vision updates for odometry are enabled. */
    public void setVisionUpdatesEnabled(boolean enabled) {
        enableVisionUpdates = enabled;
        enableVisionUpdatesAlert.set(!enabled);
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
        }

        // Loop over instances
        List<Pose2d> allRobotPoses = new ArrayList<>();
        List<Pose3d> allRobotPoses3d = new ArrayList<>();
        List<VisionObservation> allVisionObservations = new ArrayList<>();
        for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
            // Loop over frames
            for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
                lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
                var timestamp = inputs[instanceIndex].timestamps[frameIndex];
                var values = inputs[instanceIndex].frames[frameIndex];

                // Exit if blank frame
                if (values.length == 0 || values[0] == 0) {
                    continue;
                }

                // Switch based on number of poses
                Pose3d cameraPose = null;
                Pose3d robotPose3d = null;
                switch ((int) values[0]) {
                    case 1:
                        // One pose (multi-tag), use directly
                        cameraPose =
                                new Pose3d(
                                        values[2],
                                        values[3],
                                        values[4],
                                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        robotPose3d =
                                cameraPose.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());
                        break;
                    case 2:
                        // Two poses (one tag), disambiguate
                        double error0 = values[1];
                        double error1 = values[9];
                        Pose3d cameraPose0 =
                                new Pose3d(
                                        values[2],
                                        values[3],
                                        values[4],
                                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        Pose3d cameraPose1 =
                                new Pose3d(
                                        values[10],
                                        values[11],
                                        values[12],
                                        new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
                        Pose3d robotPose3d0 =
                                cameraPose0.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());
                        Pose3d robotPose3d1 =
                                cameraPose1.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());

                        // Select pose using projection errors
                        if (error0 < error1 * ambiguityThreshold) {
                            cameraPose = cameraPose0;
                            robotPose3d = robotPose3d0;
                        } else if (error1 < error0 * ambiguityThreshold) {
                            cameraPose = cameraPose1;
                            robotPose3d = robotPose3d1;
                        }
                        break;
                }

                // Exit if no data
                if (cameraPose == null || robotPose3d == null) {
                    continue;
                }

                // Exit if robot pose is off the field
                if (robotPose3d.getX() < -fieldBorderMargin
                        || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
                        || robotPose3d.getY() < -fieldBorderMargin
                        || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
                        || robotPose3d.getZ() < -zMargin
                        || robotPose3d.getZ() > zMargin) {
                    continue;
                }

                // Get 2D robot pose
                Pose2d robotPose = robotPose3d.toPose2d();

                // Get tag poses and update last detection times
                List<Pose3d> tagPoses = new ArrayList<>();
                for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
                    int tagId = (int) values[i];
                    lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                    Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) values[i]);
                    tagPose.ifPresent(tagPoses::add);
                }

                // Calculate average distance to tag
                double totalDistance = 0.0;
                for (Pose3d tagPose : tagPoses) {
                    totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                }
                double avgDistance = totalDistance / tagPoses.size();

                // Add observation to list
                double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
                double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
                allVisionObservations.add(
                        new VisionObservation(
                                robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
                allRobotPoses.add(robotPose);
                allRobotPoses3d.add(robotPose3d);

                // Log data from instance
                Logger.recordOutput(
                        "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
                        Timer.getFPGATimestamp() - timestamp);
                Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
                Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
                Logger.recordOutput(
                        "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
            }

            // If no frames from instances, clear robot pose
            if (inputs[instanceIndex].timestamps.length == 0) {
                Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", new double[] {});
                Logger.recordOutput(
                        "AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", new double[] {});
            }

            // If no recent frames from instance, clear tag poses
            if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
                Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new double[] {});
            }
        }

        // Log robot poses
        Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
        Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

        // Log tag poses
        List<Pose3d> allTagPoses = new ArrayList<>();
        for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
            if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
                allTagPoses.add(FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).get());
            }
        }
        Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

        // Send results to robot state
        if (enableVisionUpdates) {
            allVisionObservations.forEach(robotState::addVisionObservation);
        }
    }
}
