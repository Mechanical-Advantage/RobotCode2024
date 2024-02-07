package frc.robot.util.trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;

public class HolonomicDriveController {
  private final PIDController linearController;
  private final PIDController thetaController;

  private HolonomicTrajectory.State currentState = null;

  /** -- SETTER -- Set tolerance for goal state */
  @Setter private HolonomicTrajectory.State goalTolerance = null;

  private Pose2d controllerTolerance = null;

  /** -- SETTER -- Set goal state */
  @Setter private HolonomicTrajectory.State goalState = null;

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
  public ChassisSpeeds calculate(
      HolonomicTrajectory.State currentState, HolonomicTrajectory.State setpointState) {
    this.currentState = currentState;
    Pose2d setpointPose = setpointState.pose();
    poseError = setpointPose.relativeTo(currentState.pose());

    // Calculate feedback velocities (based on position error).
    double linearFeedback =
        linearController.calculate(
            0, currentState.pose().getTranslation().getDistance(setpointPose.getTranslation()));
    Rotation2d currentToStateAngle =
        setpointPose.getTranslation().minus(currentState.pose().getTranslation()).getAngle();
    double xFeedback = linearFeedback * currentToStateAngle.getCos();
    double yFeedback = linearFeedback * currentToStateAngle.getSin();
    double thetaFeedback =
        thetaController.calculate(
            currentState.pose().getRotation().getRadians(),
            setpointPose.getRotation().getRadians());
    if (atGoal()) {
      if (linearController.atSetpoint()) {
        xFeedback = 0;
        yFeedback = 0;
      }
      if (thetaController.atSetpoint()) {
        thetaFeedback = 0;
      }
    }

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        setpointState.velocityX() + xFeedback,
        setpointState.velocityY() + yFeedback,
        setpointState.angularVelocity() + thetaFeedback,
        currentState.pose().getRotation());
  }

  public boolean atSetpoint() {
    return linearController.atSetpoint() && thetaController.atSetpoint();
  }

  /** Within tolerance of current goal */
  public boolean atGoal() {
    Pose2d goalPoseError = goalState.pose().relativeTo(currentState.pose());
    boolean withinPoseTolerance =
        goalPoseError.getTranslation().getNorm() <= goalTolerance.pose().getTranslation().getNorm()
            && Math.abs(goalPoseError.getRotation().getRadians())
                <= goalTolerance.pose().getRotation().getRadians();
    boolean withinVelocityTolerance =
        Math.abs(currentState.velocityX() - goalState.velocityX()) < goalTolerance.velocityX()
            && Math.abs(currentState.velocityY() - goalState.velocityY())
                < goalTolerance.velocityY()
            && Math.abs(currentState.angularVelocity() - goalState.angularVelocity())
                < goalTolerance.angularVelocity();
    return withinPoseTolerance && withinVelocityTolerance;
  }
}
