package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import com.google.common.hash.Hashing;
import io.grpc.Grpc;
import io.grpc.InsecureChannelCredentials;
import java.io.*;
import java.math.RoundingMode;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceGrpc;

public class GenerateTrajectories {
  public static void main(String[] args) {
    Constants.disableHAL();

    // Create vehicle model
    VehicleModel model =
        VehicleModel.newBuilder()
            .setMass(70)
            .setMoi(6)
            .setVehicleLength(DriveConstants.driveConfig.trackWidthX())
            .setVehicleWidth(DriveConstants.driveConfig.trackWidthY())
            .setWheelRadius(DriveConstants.driveConfig.wheelRadius())
            .setMaxWheelTorque(6)
            .setMaxWheelOmega(
                DriveConstants.moduleLimits.maxDriveVelocity()
                    / DriveConstants.driveConfig.wheelRadius())
            .build();

    // Check hashcodes
    Map<String, List<PathSegment>> pathQueue = new HashMap<>();
    for (Map.Entry<String, List<PathSegment>> entry : DriveTrajectories.paths.entrySet()) {
      String hashCode = getHashCode(model, entry.getValue());
      File pathFile =
          Path.of("src", "main", "deploy", "trajectories", entry.getKey() + ".pathblob").toFile();
      if (pathFile.exists()) {
        try {
          InputStream fileStream = new FileInputStream(pathFile);
          Trajectory trajectory = Trajectory.parseFrom(fileStream);
          // trajectory.getStatesC
          if (!trajectory.getHashCode().equals(hashCode)) {
            pathQueue.put(entry.getKey(), entry.getValue());
          }
        } catch (IOException e) {
          e.printStackTrace();
        }
      } else {
        pathQueue.put(entry.getKey(), entry.getValue());
      }
    }

    // Exit if trajectories up-to-date
    if (pathQueue.isEmpty()) {
      return;
    }

    // Connect to service
    var channel =
        Grpc.newChannelBuilder("127.0.0.1:56328", InsecureChannelCredentials.create()).build();
    var service = VehicleTrajectoryServiceGrpc.newBlockingStub(channel);

    // Generate trajectories
    for (Map.Entry<String, List<PathSegment>> entry : pathQueue.entrySet()) {
      PathRequest request =
          PathRequest.newBuilder().setModel(model).addAllSegments(entry.getValue()).build();

      TrajectoryResponse response = service.generateTrajectory(request);
      System.out.println(response.getError());

      String responseHashCode = getHashCode(model, entry.getValue());

      response =
          response.toBuilder()
              .setTrajectory(
                  response.getTrajectory().toBuilder().setHashCode(responseHashCode).build())
              .build();

      File pathFile =
          Path.of("src", "main", "deploy", "trajectories", entry.getKey() + ".pathblob").toFile();

      try {
        OutputStream fileStream = new FileOutputStream(pathFile);
        System.out.println("Writing to " + pathFile.getAbsolutePath());
        response.getTrajectory().writeTo(fileStream);
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  // create a hashcode for the vehicle model and path segments

  private static String getHashCode(VehicleModel model, List<PathSegment> segements) {
    StringBuilder hashString = new StringBuilder();

    DecimalFormat format = new DecimalFormat("#.000000");
    format.setRoundingMode(RoundingMode.HALF_DOWN);

    hashString.append(format.format(model.getMass()));
    hashString.append(format.format(model.getMoi()));
    hashString.append(format.format(model.getVehicleLength()));
    hashString.append(format.format(model.getVehicleWidth()));
    hashString.append(format.format(model.getWheelRadius()));
    hashString.append(format.format(model.getMaxWheelTorque()));
    hashString.append(format.format(model.getMaxWheelOmega()));

    for (PathSegment segment : segements) {
      for (Waypoint waypoint : segment.getWaypointsList()) {
        hashString.append(format.format(waypoint.getX()));
        hashString.append(format.format(waypoint.getY()));
        if (waypoint.hasHeadingConstraint()) {
          hashString.append(format.format(waypoint.getHeadingConstraint()));
        }

        if (waypoint.hasSamples()) {
          hashString.append(waypoint.getSamples());
        }

        switch (waypoint.getVelocityConstraintCase()) {
          case ZERO_VELOCITY -> {
            hashString.append(format.format(0));
          }
          case VEHICLE_VELOCITY -> {
            hashString.append(format.format(waypoint.getVehicleVelocity().getVx()));
            hashString.append(format.format(waypoint.getVehicleVelocity().getVy()));
            hashString.append(format.format(waypoint.getVehicleVelocity().getOmega()));
          }
        }
      }

      if (segment.hasMaxVelocity()) {
        hashString.append(format.format(segment.getMaxVelocity()));
      }
      if (segment.hasMaxOmega()) {
        hashString.append(format.format(segment.getMaxOmega()));
      }

      hashString.append(segment.getStraightLine());
    }

    return Hashing.sha256().hashString(hashString, StandardCharsets.UTF_8).toString();
  }
}
