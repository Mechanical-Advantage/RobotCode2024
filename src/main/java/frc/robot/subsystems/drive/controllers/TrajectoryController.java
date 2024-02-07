package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.trajectoryConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.HolonomicDriveController;
import frc.robot.util.trajectory.HolonomicTrajectory;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {
  private static LoggedTunableNumber trajectoryLinearkP =
      new LoggedTunableNumber("Trajectory/linearkP", trajectoryConstants.linearkP());
  private static LoggedTunableNumber trajectoryLinearkD =
      new LoggedTunableNumber("Trajectory/linearkD", trajectoryConstants.linearkD());
  private static LoggedTunableNumber trajectoryThetakP =
      new LoggedTunableNumber("Trajectory/thetakP", trajectoryConstants.thetakP());
  private static LoggedTunableNumber trajectoryThetakD =
      new LoggedTunableNumber("Trajectory/thetakD", trajectoryConstants.thetakD());
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

  private HolonomicDriveController controller;
  private HolonomicTrajectory trajectory;
  private double startTime;

  public TrajectoryController(HolonomicTrajectory trajectory) {
    this.trajectory = trajectory;
    controller =
        new HolonomicDriveController(
            trajectoryLinearkP.get(),
            trajectoryLinearkD.get(),
            trajectoryThetakP.get(),
            trajectoryThetakD.get());
    controller.setGoalState(AllianceFlipUtil.apply(trajectory.getEndState()));
    configTrajectoryTolerances();
    startTime = Timer.getFPGATimestamp();
    // Log poses
    Logger.recordOutput(
        "Trajectory/trajectoryPoses",
        Arrays.stream(trajectory.getTrajectoryPoses())
            .map(AllianceFlipUtil::apply)
            .toArray(Pose2d[]::new));
  }

  public ChassisSpeeds update() {
    // Update PID Controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> controller.setPID(pid[0], pid[1], pid[2], pid[3]),
        trajectoryLinearkP,
        trajectoryLinearkD,
        trajectoryThetakP,
        trajectoryThetakP);
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

    // Run trajectory
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
    double sampletime = Timer.getFPGATimestamp() - startTime;
    HolonomicTrajectory.State currentState =
        new HolonomicTrajectory.State(
            sampletime, currentPose, fieldVelocity.dx, fieldVelocity.dy, fieldVelocity.dtheta);
    // Sample and flip state
    HolonomicTrajectory.State setpointState = trajectory.sample(sampletime);
    setpointState = AllianceFlipUtil.apply(setpointState);
    // calculate trajectory speeds
    ChassisSpeeds speeds = controller.calculate(currentState, setpointState);
    // Log trajectory data
    Logger.recordOutput("Trajectory/SetpointPose", setpointState.pose());
    Logger.recordOutput("Trajectory/FieldRelativeSpeeds", speeds);
    return speeds;
  }

  @AutoLogOutput(key = "Trajectory/TranslationError")
  public double getTranslationError() {
    return controller.getPoseError().getTranslation().getNorm();
  }

  @AutoLogOutput(key = "Trajectory/RotationError")
  public Rotation2d getRotationError() {
    return controller.getPoseError().getRotation();
  }

  @AutoLogOutput(key = "Trajectory/Finished")
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > trajectory.getDuration()
        && controller.atGoal()
        && controller.atSetpoint();
  }

  private void configTrajectoryTolerances() {
    controller.setControllerTolerance(
        new Pose2d(
            new Translation2d(
                trajectoryLinearTolerance.get(), Rotation2d.fromRadians(Math.PI / 4.0)),
            Rotation2d.fromRadians(trajectoryThetaTolerance.get())));
    double velocityXTolerance = trajectoryLinearVelocityTolerance.get() / Math.sqrt(2.0);
    controller.setGoalTolerance(
        new HolonomicTrajectory.State(
            0.0,
            new Pose2d(
                new Translation2d(
                    trajectoryGoalLinearTolerance.get(), Rotation2d.fromRadians(Math.PI / 4.0)),
                Rotation2d.fromRadians(trajectoryGoalThetaTolerance.get())),
            velocityXTolerance,
            velocityXTolerance, // Same value
            trajectoryAngularVelocityTolerance.get()));
  }
}
