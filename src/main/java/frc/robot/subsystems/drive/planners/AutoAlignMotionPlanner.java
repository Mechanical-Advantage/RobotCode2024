package frc.robot.subsystems.drive.planners;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignMotionPlanner {
  private static LoggedTunableNumber linearKp =
      new LoggedTunableNumber("AutoAlign/driveKp", DriveConstants.autoAlignConstants.linearKp());
  private static LoggedTunableNumber linearKd =
      new LoggedTunableNumber("AutoAlign/driveKd", DriveConstants.autoAlignConstants.linearKd());
  private static LoggedTunableNumber thetaKp =
      new LoggedTunableNumber("AutoAlign/thetaKp", DriveConstants.autoAlignConstants.thetaKp());
  private static LoggedTunableNumber thetaKd =
      new LoggedTunableNumber("AutoAlign/thetaKd", DriveConstants.autoAlignConstants.thetaKd());
  private static LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber(
          "AutoAlign/controllerLinearTolerance",
          DriveConstants.autoAlignConstants.linearTolerance());
  private static LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber(
          "AutoAlign/controllerThetaTolerance", DriveConstants.autoAlignConstants.thetaTolerance());
  private static LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxLinearVelocity", DriveConstants.autoAlignConstants.maxLinearVelocity());
  private static LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxLinearAcceleration",
          DriveConstants.autoAlignConstants.maxLinearAcceleration());
  private static LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularVelocity", DriveConstants.autoAlignConstants.maxAngularVelocity());
  private static LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularAcceleration",
          DriveConstants.autoAlignConstants.maxAngularAcceleration());

  @Getter(onMethod_ = @AutoLogOutput(key = "AutoAlign/goalPose"))
  private Pose2d goalPose = null;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;

  // Store previous velocities for acceleration limiting
  private Translation2d prevLinearVelocity;
  private double prevAngularVelocity;

  public AutoAlignMotionPlanner() {
    linearController =
        new ProfiledPIDController(
            linearKp.get(),
            0,
            linearKd.get(),
            new TrapezoidProfile.Constraints(maxLinearVelocity.get(), maxLinearAcceleration.get()));
    linearController.setTolerance(linearTolerance.get());

    thetaController =
        new ProfiledPIDController(
            thetaKp.get(),
            0,
            thetaKd.get(),
            new TrapezoidProfile.Constraints(
                maxAngularVelocity.get(), maxAngularAcceleration.get()));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance.get());
  }

  public void setGoalPose(Pose2d goalPose) {
    this.goalPose = goalPose;

    // Reset controllers
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
    // Linear controller will control to 0 so distance is the measurement
    Rotation2d rotationToGoal =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double velocity =
        -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
            .rotateBy(rotationToGoal.unaryMinus())
            .getX();
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()), velocity);
    linearController.setGoal(0.0);
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);

    prevLinearVelocity = new Translation2d(fieldVelocity.dx, fieldVelocity.dy);
    prevAngularVelocity = fieldVelocity.dtheta;
  }

  public ChassisSpeeds update() {
    updateControllers();
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();

    // Calculate feedback velocities (based on position error).
    double linearVelocityScalar =
        linearController.calculate(
                currentPose.getTranslation().getDistance(goalPose.getTranslation()))
            + linearController.getSetpoint().velocity;
    Rotation2d rotationToGoal =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double xVelocity = -linearVelocityScalar * rotationToGoal.getCos();
    double yVelocity = -linearVelocityScalar * rotationToGoal.getSin();

    // If current angular velocity is fast enough in current direction continue in direction
    double angularVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians())
            + thetaController.getSetpoint().velocity;

    // Limit linear acceleration
    // Forward limiting and brake limiting
    Translation2d desiredLinearVelocity = new Translation2d(xVelocity, yVelocity);
    Translation2d deltaVelocity = desiredLinearVelocity.minus(prevLinearVelocity);
    double maxDeltaVelocity = maxLinearAcceleration.get() * 0.02;
    if (deltaVelocity.getNorm() > maxDeltaVelocity) {
      desiredLinearVelocity =
          prevLinearVelocity.plus(deltaVelocity.times(maxDeltaVelocity / deltaVelocity.getNorm()));
    }
    prevLinearVelocity = new Translation2d(fieldVelocity.dx, fieldVelocity.dy);

    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    Logger.recordOutput("AutoAlign/speeds", fieldRelativeSpeeds);
    Logger.recordOutput("AutoAlign/linearError", linearController.getPositionError());
    Logger.recordOutput("AutoAlign/thetaError", thetaController.getPositionError());
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/atGoal")
  public boolean atGoal() {
    return linearController.atGoal() && thetaController.atSetpoint();
  }

  private void updateControllers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearKp.get(), 0, linearKd.get()),
        linearKp,
        linearKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetaKp.get(), 0, thetaKd.get()),
        thetaKp,
        thetaKd);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> thetaController.setTolerance(thetaTolerance.get()), thetaTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            linearController.setConstraints(
                new TrapezoidProfile.Constraints(
                    maxLinearVelocity.get(), maxLinearAcceleration.get())),
        maxLinearVelocity,
        maxLinearAcceleration);
  }
}
