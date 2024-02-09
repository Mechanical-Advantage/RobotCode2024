package org.littletonrobotics.frc2024.subsystems.drive.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import io.grpc.Grpc;
import io.grpc.InsecureChannelCredentials;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceGrpc;

public class GenerateTrajectories {
    public static void main(String[] args) {
        Constants.disableHAL();
        var channel =
                Grpc.newChannelBuilder("10.63.28.184:56328", InsecureChannelCredentials.create()).build();
        var service = VehicleTrajectoryServiceGrpc.newBlockingStub(channel);

        VehicleModel model =
                VehicleModel.newBuilder()
                        .setMass(70)
                        .setMoi(6)
                        .setVehicleLength(DriveConstants.driveConfig.trackwidthX())
                        .setVehicleWidth(DriveConstants.driveConfig.trackwidthY())
                        .setWheelRadius(DriveConstants.driveConfig.wheelRadius())
                        .setMaxWheelTorque(2)
                        .setMaxWheelOmega(
                                DriveConstants.moduleLimits.maxDriveVelocity()
                                        / DriveConstants.driveConfig.wheelRadius())
                        .build();

        PathRequest request =
                PathRequest.newBuilder()
                        .setModel(model)
                        .addAllSegments(DriveTrajectories.driveStraight)
                        .build();

        TrajectoryResponse response = service.generateTrajectory(request);
        System.out.println(response.toString());
    }
}
