package org.littletonrobotics.frc2024.util.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;

public class HolonomicDriveController {
    private final PIDController linearController;
    private final PIDController thetaController;

    @Getter private Pose2d poseError;

    public HolonomicDriveController(
            double linearkP, double linearkD, double thetakP, double thetakD) {
        linearController = new PIDController(linearkP, 0, linearkD);
        thetaController = new PIDController(thetakP, 0, thetakD);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** Reset all controllers */
    public void resetControllers() {
        linearController.reset();
        thetaController.reset();
    }

    public void resetThetaController() {
        thetaController.reset();
    }

    public void setControllerTolerance(Pose2d controllerTolerance) {
        linearController.setTolerance(controllerTolerance.getTranslation().getNorm());
        thetaController.setTolerance(controllerTolerance.getRotation().getRadians());
    }

    /** Set PID values */
    public void setPID(double linearkP, double linearkD, double thetakP, double thetakD) {
        linearController.setPID(linearkP, 0, linearkD);
        thetaController.setPID(thetakP, 0, thetakD);
    }

    /** Calculate robot relative chassis speeds */
    public ChassisSpeeds calculate(VehicleState currentState, VehicleState setpointState) {
        Pose2d currentPose =
                new Pose2d(
                        currentState.getX(), currentState.getY(), new Rotation2d(currentState.getTheta()));
        Pose2d setpointPose =
                new Pose2d(
                        setpointState.getX(), setpointState.getY(), new Rotation2d(setpointState.getTheta()));
        poseError = setpointPose.relativeTo(currentPose);

        // Calculate feedback velocities (based on position error).
        double linearFeedback =
                linearController.calculate(
                        0, currentPose.getTranslation().getDistance(setpointPose.getTranslation()));
        Rotation2d currentToStateAngle =
                setpointPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        double xFeedback = linearFeedback * currentToStateAngle.getCos();
        double yFeedback = linearFeedback * currentToStateAngle.getSin();
        double thetaFeedback =
                thetaController.calculate(
                        currentPose.getRotation().getRadians(), setpointPose.getRotation().getRadians());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                setpointState.getVx() + xFeedback,
                setpointState.getVy() + yFeedback,
                setpointState.getOmega() + thetaFeedback,
                currentPose.getRotation());
    }
}
