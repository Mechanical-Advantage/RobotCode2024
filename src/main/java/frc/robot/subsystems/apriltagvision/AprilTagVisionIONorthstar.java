package frc.robot.subsystems.apriltagvision;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.util.Alert;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
    private static final int cameraId = 0;
    private static final int cameraResolutionWidth = 1600;
    private static final int cameraResolutionHeight = 1200;
    private static final int cameraAutoExposure = 1;
    private static final int cameraExposure = 10;
    private static final int cameraGain = 25;

    private final DoubleArraySubscriber observationSubscriber;
    private final DoubleArraySubscriber demoObservationSubscriber;
    private final IntegerSubscriber fpsSubscriber;

    private static final double disconnectedTimeout = 0.5;
    private final Alert disconnectedAlert;
    private final Timer disconnectedTimer = new Timer();
    private final String identifier;

    public AprilTagVisionIONorthstar(String identifier) {
        this.identifier = identifier;
        System.out.println("[Init] Creating AprilTagVisionIONorthstar (" + identifier + ")");
        var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);

        var configTable = northstarTable.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(cameraId);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
        try {
            configTable
                    .getStringTopic("tag_layout")
                    .publish()
                    .set(new ObjectMapper().writeValueAsString(FieldConstants.aprilTags));
        } catch (JsonProcessingException e) {
            throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
        }

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber =
                outputTable
                        .getDoubleArrayTopic("observations")
                        .subscribe(
                                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        demoObservationSubscriber =
                outputTable
                        .getDoubleArrayTopic("demo_observations")
                        .subscribe(
                                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

        disconnectedAlert = new Alert("No data from \"" + identifier + "\"", Alert.AlertType.ERROR);
        disconnectedTimer.start();
    }

    private int outOfOrderCounter = 0;
    private double prevLatestTime = 0;

    public void updateInputs(AprilTagVisionIOInputs inputs) {
        var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
        inputs.demoFrame = new double[] {};
        for (double[] demoFrame : demoObservationSubscriber.readQueueValues()) {
            inputs.demoFrame = demoFrame;
        }
        inputs.fps = fpsSubscriber.get();

        // Update disconnected alert
        if (queue.length > 0) {
            disconnectedTimer.reset();
        }
        disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));

        // Log out of order
        double[] orderedTimestamps = inputs.timestamps.clone();
        Arrays.sort(orderedTimestamps);
        for (int i = 0; i < inputs.timestamps.length; i++) {
            if (orderedTimestamps[i] != inputs.timestamps[i]) {
                outOfOrderCounter++;
            }
        }

        if (orderedTimestamps.length > 0) {
            for (double timestamp : inputs.timestamps) {
                if (timestamp < prevLatestTime) {
                    outOfOrderCounter++;
                }
            }
            if (orderedTimestamps[orderedTimestamps.length - 1] > prevLatestTime) {
                prevLatestTime = orderedTimestamps[orderedTimestamps.length - 1];
            }
        }
        Logger.recordOutput("AprilTagVision/OutOfOrderCounter_" + identifier, outOfOrderCounter);
    }
}
