package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.trajectory.HolonomicDriveController.HolonomicDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.HolonomicDriveController;
import frc.robot.util.trajectory.Trajectory;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class AutoMotionPlanner {
  private static LoggedTunableNumber trajectoryLinearKp =
      new LoggedTunableNumber("Trajectory/linearKp", trajectoryConstants.linearKp());
  private static LoggedTunableNumber trajectoryLinearKd =
      new LoggedTunableNumber("Trajectory/linearKd", trajectoryConstants.linearKd());
  private static LoggedTunableNumber trajectoryThetaKp =
      new LoggedTunableNumber("Trajectory/thetaKp", trajectoryConstants.thetaKp());
  private static LoggedTunableNumber trajectoryThetaKd =
      new LoggedTunableNumber("Trajectory/thetaKd", trajectoryConstants.thetaKd());
  private static LoggedTunableNumber trajectoryLinearTolerance =
      new LoggedTunableNumber(
          "Trajectory/controllerLinearTolerance", trajectoryConstants.linearTolerance());
  private static LoggedTunableNumber trajectoryThetaTolerance =
      new LoggedTunableNumber(
          "Trajectory/controllerThetaTolerance", trajectoryConstants.thetaTolerance());
  private static LoggedTunableNumber trajectoryGoalLinearTolerance =
      new LoggedTunableNumber(
          "Trajectory/goalLinearTolerance", trajectoryConstants.goalLinearTolerance());
  private static LoggedTunableNumber trajectoryGoalThetaTolerance =
      new LoggedTunableNumber(
          "Trajectory/goalThetaTolerance", trajectoryConstants.goalThetaTolerance());
  private static LoggedTunableNumber trajectoryLinearVelocityTolerance =
      new LoggedTunableNumber(
          "Trajectory/goalLinearVelocityTolerance", trajectoryConstants.linearVelocityTolerance());
  private static LoggedTunableNumber trajectoryAngularVelocityTolerance =
      new LoggedTunableNumber(
          "Trajectory/goalAngularVelocityTolerance",
          trajectoryConstants.angularVelocityTolerance());

  private Trajectory trajectory = null;
  private Double startTime = null;
  private Double currentTime = null;
  private final HolonomicDriveController controller;

  public AutoMotionPlanner() {
    controller =
        new HolonomicDriveController(
            trajectoryLinearKp.get(),
            trajectoryLinearKd.get(),
            trajectoryThetaKp.get(),
            trajectoryThetaKd.get());
    configTrajectoryTolerances();
  }

  protected void setTrajectory(Trajectory trajectory) {
    // Only set if not following trajectory or done with current one
    if (this.trajectory == null || isFinished()) {
      this.trajectory = trajectory;
      controller.setGoalState(trajectory.getEndState());
      controller.resetControllers();
      startTime = null;
      currentTime = null;
      // Log poses
      Logger.recordOutput(
          "Trajectory/trajectoryPoses",
          Arrays.stream(trajectory.getTrajectoryPoses())
              .map(AllianceFlipUtil::apply)
              .toArray(Pose2d[]::new));
    }
  }

  /** Output setpoint chassis speeds in */
  protected ChassisSpeeds update(double timestamp, Pose2d robot, Twist2d fieldVelocity) {
    System.out.println(timestamp + " " + robot.toString() + " " + fieldVelocity.toString());
    // If disabled reset everything and stop
    if (DriverStation.isDisabled()) {
      trajectory = null;
      // Stop logs
      Logger.recordOutput("Trajectory/trajectoryPoses", new Pose2d[] {});
      Logger.recordOutput("Trajectory/setpointPose", new Pose2d());
      Logger.recordOutput("Trajectory/speeds", new double[] {});
      return new ChassisSpeeds();
    }

    // Tunable numbers
    if (Constants.tuningMode) {
      // Update PID Controllers
      LoggedTunableNumber.ifChanged(
          hashCode(),
          pid -> controller.setPID(pid[0], pid[1], pid[2], pid[3]),
          trajectoryLinearKp,
          trajectoryLinearKd,
          trajectoryThetaKp,
          trajectoryThetaKp);
      // Tolerances
      LoggedTunableNumber.ifChanged(
          hashCode(),
          this::configTrajectoryTolerances,
          trajectoryLinearTolerance,
          trajectoryThetaTolerance,
          trajectoryGoalLinearTolerance,
          trajectoryGoalThetaTolerance,
          trajectoryLinearVelocityTolerance,
          trajectoryAngularVelocityTolerance);
    }

    // Reached end
    if (isFinished() || trajectory == null) {
      // Stop logs
      Logger.recordOutput("Trajectory/trajectoryPoses", new Pose2d[] {});
      Logger.recordOutput("Trajectory/setpointPose", new Pose2d());
      Logger.recordOutput("Trajectory/speeds", new double[] {});
      return new ChassisSpeeds();
    }

    if (startTime == null) {
      startTime = timestamp;
    }
    currentTime = timestamp;

    HolonomicDriveState currentState =
        new HolonomicDriveState(robot, fieldVelocity.dx, fieldVelocity.dy, fieldVelocity.dtheta);
    // Sample and flip state
    HolonomicDriveState setpointState = trajectory.sample(currentTime - startTime);
    setpointState = AllianceFlipUtil.apply(setpointState);
    // calculate trajectory speeds
    ChassisSpeeds speeds = controller.calculate(currentState, setpointState);

    // Log trajectory data
    Logger.recordOutput("Trajectory/setpointPose", setpointState.pose());
    Logger.recordOutput(
        "Trajectory/translationError", controller.getPoseError().getTranslation().getNorm());
    Logger.recordOutput(
        "Trajectory/rotationError", controller.getPoseError().getRotation().getRadians());
    return speeds;
  }

  protected boolean isFinished() {
    return trajectory != null
        && currentTime != null
        && startTime != null
        && currentTime - startTime >= trajectory.getDuration()
        && controller.atGoal();
  }

  private void configTrajectoryTolerances() {
    controller.setControllerTolerance(
        new Pose2d(
            new Translation2d(
                trajectoryLinearTolerance.get(), Rotation2d.fromRadians(Math.PI / 4.0)),
            Rotation2d.fromRadians(trajectoryThetaTolerance.get())));
    double velocityXTolerance = trajectoryLinearVelocityTolerance.get() / Math.sqrt(2.0);
    controller.setGoalTolerance(
        new HolonomicDriveState(
            new Pose2d(
                new Translation2d(
                    trajectoryGoalLinearTolerance.get(), Rotation2d.fromRadians(Math.PI / 4.0)),
                Rotation2d.fromRadians(trajectoryGoalThetaTolerance.get())),
            velocityXTolerance,
            velocityXTolerance, // Same value
            trajectoryAngularVelocityTolerance.get()));
  }
}
