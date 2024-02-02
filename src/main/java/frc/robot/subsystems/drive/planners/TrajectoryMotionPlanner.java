package frc.robot.subsystems.drive.planners;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.trajectory.HolonomicDriveController.HolonomicDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.HolonomicDriveController;
import frc.robot.util.trajectory.Trajectory;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectoryMotionPlanner {
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
  private Timer trajectoryTimer = new Timer();
  private final HolonomicDriveController controller;

  public TrajectoryMotionPlanner() {
    controller =
        new HolonomicDriveController(
            trajectoryLinearKp.get(),
            trajectoryLinearKd.get(),
            trajectoryThetaKp.get(),
            trajectoryThetaKd.get());
    configTrajectoryTolerances();
  }

  public void setTrajectory(Trajectory trajectory) {
    this.trajectory = trajectory;
    controller.setGoalState(AllianceFlipUtil.apply(trajectory.getEndState()));
    controller.resetControllers();
    trajectoryTimer.restart();
    // Log poses
    Logger.recordOutput(
        "Trajectory/trajectoryPoses",
        Arrays.stream(trajectory.getTrajectoryPoses())
            .map(AllianceFlipUtil::apply)
            .toArray(Pose2d[]::new));
  }

  /** Output setpoint chassis speeds in */
  public ChassisSpeeds update() {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
    // If disabled reset everything and stop
    if (DriverStation.isDisabled()) {
      trajectory = null;
      trajectoryTimer.stop();
      trajectoryTimer.reset();
      controller.resetControllers();
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

    HolonomicDriveState currentState =
        new HolonomicDriveState(
            currentPose, fieldVelocity.dx, fieldVelocity.dy, fieldVelocity.dtheta);
    // Sample and flip state
    HolonomicDriveState setpointState = trajectory.sample(trajectoryTimer.get());
    setpointState = AllianceFlipUtil.apply(setpointState);
    // calculate trajectory speeds
    ChassisSpeeds speeds = controller.calculate(currentState, setpointState);
    // Reached end
    if (isFinished()) {
      trajectoryTimer.stop();
      // Stop logs
      Logger.recordOutput("Trajectory/trajectoryPoses", new Pose2d[] {});
      Logger.recordOutput("Trajectory/setpointPose", new Pose2d());
      Logger.recordOutput("Trajectory/speeds", new double[] {});
      return new ChassisSpeeds();
    }
    // Log trajectory data
    Logger.recordOutput("Trajectory/setpointPose", setpointState.pose());
    Logger.recordOutput(
        "Trajectory/translationError", controller.getPoseError().getTranslation().getNorm());
    Logger.recordOutput(
        "Trajectory/rotationError", controller.getPoseError().getRotation().getRadians());
    Logger.recordOutput("Trajectory/speeds", speeds);
    return speeds;
  }

  @AutoLogOutput(key = "Trajectory/finished")
  public boolean isFinished() {
    return trajectory != null
        && trajectoryTimer.hasElapsed(trajectory.getDuration())
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
