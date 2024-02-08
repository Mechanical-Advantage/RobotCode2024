package org.littletonrobotics.frc2024.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

/** Drive controller for outputting {@link ChassisSpeeds} from driver joysticks. */
public class TeleopDriveController {
    private static final LoggedTunableNumber controllerDeadband =
            new LoggedTunableNumber("TeleopDrive/Deadband", 0.1);

    private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;

    /**
     * Accepts new drive input from joysticks.
     *
     * @param x Desired x velocity scalar, -1..1
     * @param y Desired y velocity scalar, -1..1
     * @param omega Desired omega velocity scalar, -1..1
     */
    public void acceptDriveInput(double x, double y, double omega) {
        controllerX = x;
        controllerY = y;
        controllerOmega = omega;
    }

    /**
     * Updates the controller with the currently stored state.
     *
     * @return {@link ChassisSpeeds} with driver's requested speeds.
     */
    public ChassisSpeeds update() {
        // Apply deadband
        double linearMagnitude =
                MathUtil.applyDeadband(Math.hypot(controllerX, controllerY), controllerDeadband.get());
        Rotation2d linearDirection = new Rotation2d(controllerX, controllerY);
        double omega = MathUtil.applyDeadband(controllerOmega, controllerDeadband.get());

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = Math.copySign(omega * omega, omega);

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * DriveConstants.driveConfig.maxLinearVelocity(),
                linearVelocity.getY() * DriveConstants.driveConfig.maxLinearVelocity(),
                omega * DriveConstants.driveConfig.maxAngularVelocity(),
                RobotState.getInstance().getEstimatedPose().getRotation());
    }
}
