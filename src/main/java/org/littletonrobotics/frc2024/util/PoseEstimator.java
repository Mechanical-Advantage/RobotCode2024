// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package org.littletonrobotics.frc2024.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class PoseEstimator {
    private static final double historyLengthSecs = 0.3;

    private Pose2d basePose = new Pose2d();

    /** -- GETTER -- Returns the latest robot pose based on drive and vision data. */
    @Getter private Pose2d latestPose = new Pose2d();

    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();
    private final Matrix<N3, N1> q = new Matrix<>(Nat.N3(), Nat.N1());

    public PoseEstimator(Matrix<N3, N1> stateStdDevs) {
        for (int i = 0; i < 3; ++i) {
            q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    /** Resets the odometry to a known pose. */
    public void resetPose(Pose2d pose) {
        basePose = pose;
        updates.clear();
        update();
    }

    /** Records a new drive movement. */
    public void addDriveData(double timestamp, Twist2d twist) {
        updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
        update();
    }

    /** Records a new set of vision updates. */
    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
        for (var timestampedVisionUpdate : visionData) {
            var timestamp = timestampedVisionUpdate.timestamp();
            var visionUpdate =
                    new VisionUpdate(timestampedVisionUpdate.pose(), timestampedVisionUpdate.stdDevs());

            if (updates.containsKey(timestamp)) {
                // There was already an update at this timestamp, add to it
                var oldVisionUpdates = updates.get(timestamp).visionUpdates();
                oldVisionUpdates.add(visionUpdate);
                oldVisionUpdates.sort(VisionUpdate.compareDescStdDev);

            } else {
                // Insert a new update
                var prevUpdate = updates.floorEntry(timestamp);
                var nextUpdate = updates.ceilingEntry(timestamp);
                if (prevUpdate == null || nextUpdate == null) {
                    // Outside the range of existing data
                    return;
                }

                // Create partial twists (prev -> vision, vision -> next)
                var twist0 =
                        nextUpdate
                                .getValue()
                                .twist()
                                .multiply(
                                        (timestamp - prevUpdate.getKey())
                                                / (nextUpdate.getKey() - prevUpdate.getKey()));
                var twist1 =
                        nextUpdate
                                .getValue()
                                .twist()
                                .multiply(
                                        (nextUpdate.getKey() - timestamp)
                                                / (nextUpdate.getKey() - prevUpdate.getKey()));

                // Add new pose updates
                var newVisionUpdates = new ArrayList<VisionUpdate>();
                newVisionUpdates.add(visionUpdate);
                newVisionUpdates.sort(VisionUpdate.compareDescStdDev);
                updates.put(timestamp, new PoseUpdate(twist0, newVisionUpdates));
                updates.put(
                        nextUpdate.getKey(), new PoseUpdate(twist1, nextUpdate.getValue().visionUpdates()));
            }
        }

        // Recalculate latest pose once
        update();
    }

    /** Clears old data and calculates the latest pose. */
    private void update() {
        // Clear old data and update base pose
        while (updates.size() > 1
                && updates.firstKey() < Timer.getFPGATimestamp() - historyLengthSecs) {
            var update = updates.pollFirstEntry();
            basePose = update.getValue().apply(basePose, q);
        }

        // Update latest pose
        latestPose = basePose;
        for (var updateEntry : updates.entrySet()) {
            latestPose = updateEntry.getValue().apply(latestPose, q);
        }
    }

    /**
     * Represents a sequential update to a pose estimate, with a twist (drive movement) and list of
     * vision updates.
     */
    private record PoseUpdate(Twist2d twist, ArrayList<VisionUpdate> visionUpdates) {
        public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {
            // Apply drive twist
            var pose = lastPose.exp(twist);

            // Apply vision updates
            for (var visionUpdate : visionUpdates) {
                // Calculate Kalman gains based on std devs
                // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
                Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
                var r = new double[3];
                for (int i = 0; i < 3; ++i) {
                    r[i] = visionUpdate.stdDevs().get(i, 0) * visionUpdate.stdDevs().get(i, 0);
                }
                for (int row = 0; row < 3; ++row) {
                    if (q.get(row, 0) == 0.0) {
                        visionK.set(row, row, 0.0);
                    } else {
                        visionK.set(
                                row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
                    }
                }

                // Calculate twist between current and vision pose
                var visionTwist = pose.log(visionUpdate.pose());

                // Multiply by Kalman gain matrix
                var twistMatrix =
                        visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));

                // Apply twist
                pose =
                        pose.exp(
                                new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
            }

            return pose;
        }
    }

    /** Represents a single vision pose with associated standard deviations. */
    public record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
        public static final Comparator<VisionUpdate> compareDescStdDev =
                (VisionUpdate a, VisionUpdate b) -> {
                    return -Double.compare(
                            a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
                            b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
                };
    }

    /** Represents a single vision pose with a timestamp and associated standard deviations. */
    public record TimestampedVisionUpdate(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}
}
