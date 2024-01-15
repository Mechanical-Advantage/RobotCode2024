package frc.robot.util.trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicDriveController {
  private PIDController linearController, thetaController;

  private Pose2d m_poseError;
  private Rotation2d m_rotationError;

  public HolonomicDriveController(PIDController linearController, PIDController thetaController) {
    this.linearController = linearController;
    this.thetaController = thetaController;

    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds calculate(Pose2d currentPose, HolonomicDriveState state) {
    Pose2d poseRef = state.pose();

    m_poseError = poseRef.relativeTo(currentPose);
    m_rotationError = poseRef.getRotation().minus(currentPose.getRotation());

    // Calculate feedback velocities (based on position error).
    Translation2d currToStateTranslation =
        poseRef.getTranslation().minus(currentPose.getTranslation());
    double linearFeedback =
        linearController.calculate(
            0, currentPose.getTranslation().getDistance(poseRef.getTranslation()));
    double xFeedback = linearFeedback * (currToStateTranslation.getAngle().getCos());
    double yFeedback = linearFeedback * (currToStateTranslation.getAngle().getSin());
    double thetaFeedback =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), poseRef.getRotation().getRadians());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        state.velocityX() + xFeedback,
        state.velocityY() + yFeedback,
        state.angularVelocity() + thetaFeedback,
        currentPose.getRotation());
  }

  public Pose2d getPoseError() {
    return m_poseError;
  }

  public Rotation2d getRotationError() {
    return m_rotationError;
  }

  public record HolonomicDriveState(
      Pose2d pose, double velocityX, double velocityY, double angularVelocity) {}
}
