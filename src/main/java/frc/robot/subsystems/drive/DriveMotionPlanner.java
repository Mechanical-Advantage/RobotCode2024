package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.headingControllerConstants;
import static frc.robot.subsystems.drive.DriveConstants.trajectoryConstants;
import static frc.robot.util.trajectory.HolonomicDriveController.HolonomicDriveState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.HolonomicDriveController;
import frc.robot.util.trajectory.Trajectory;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveMotionPlanner {
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
  private static LoggedTunableNumber headingKp =
      new LoggedTunableNumber("Drive/headingKp", headingControllerConstants.Kp());
  private static LoggedTunableNumber headingKd =
      new LoggedTunableNumber("Drive/headingKd", headingControllerConstants.Kd());

  private ChassisSpeeds driveInputSpeeds = new ChassisSpeeds();
  private Optional<Trajectory> currentTrajectory = Optional.empty();
  private Optional<Double> trajectoryStartTime = Optional.empty();
  private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();

  private final HolonomicDriveController trajectoryController;
  private final PIDController headingController;

  private boolean xEnabled = false;

  public DriveMotionPlanner() {
    trajectoryController =
        new HolonomicDriveController(
            trajectoryLinearKp.get(),
            trajectoryLinearKd.get(),
            trajectoryThetaKp.get(),
            trajectoryThetaKd.get());
    configTrajectoryTolerances();
    headingController = new PIDController(headingKp.get(), 0, headingKd.get());
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setTrajectory(Trajectory trajectory) {
    // trajectory already running
    if (currentTrajectory.isEmpty()) {
      currentTrajectory = Optional.of(trajectory);
      trajectoryController.setGoalState(trajectory.getEndState());
      // Log poses
      Logger.recordOutput(
          "Trajectory/trajectoryPoses",
          Arrays.stream(trajectory.getTrajectoryPoses())
              .map(AllianceFlipUtil::apply)
              .toArray(Pose2d[]::new));
    }
  }

  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.ofNullable(headingSupplier);
  }

  public void disableHeadingSupplier() {
    headingSupplier = Optional.empty();
  }

  public void acceptDriveInput(ChassisSpeeds driveInputSpeeds) {
    this.driveInputSpeeds = driveInputSpeeds;
  }

  public void requestEnableXMode() {
    xEnabled = true;
  }

  public void requestDisableXMode() {
    xEnabled = false;
  }

  public ChassisSpeeds update(double timestamp, Pose2d robot, ChassisSpeeds fieldVelocity) {
    // If disabled reset everything and stop
    if (DriverStation.isDisabled()) {
      currentTrajectory = Optional.empty();
      trajectoryStartTime = Optional.empty();
      headingSupplier = Optional.empty();

      // Stop logs
      Logger.recordOutput("Trajectory/trajectoryPoses", new Pose2d[] {});
      Logger.recordOutput("Trajectory/setpointPose", new Pose2d());
      Logger.recordOutput("Drive/headingControl", new Rotation2d());
      Logger.recordOutput("Drive/speeds", new double[] {});
      return new ChassisSpeeds();
    }

    // Tunable numbers
    if (Constants.tuningMode) {
      // Update PID Controllers
      LoggedTunableNumber.ifChanged(
          hashCode(),
          pid -> trajectoryController.setPID(pid[0], pid[1], pid[2], pid[3]),
          trajectoryLinearKp,
          trajectoryLinearKd,
          trajectoryThetaKp,
          trajectoryThetaKp);
      LoggedTunableNumber.ifChanged(
          hashCode(), pid -> headingController.setPID(pid[0], 0, pid[1]), headingKp, headingKd);
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

    // Follow trajectory
    ChassisSpeeds speeds =
        currentTrajectory
            .map(
                trajectory -> {
                  // First update with trajectory
                  if (trajectoryStartTime.isEmpty()) {
                    trajectoryStartTime = Optional.of(timestamp);
                    trajectoryController.resetControllers();
                  }

                  double sampleTime = timestamp - trajectoryStartTime.get();
                  // Reached end
                  if (sampleTime > trajectory.getDuration() && trajectoryController.atGoal()) {
                    currentTrajectory = Optional.empty();
                    trajectoryStartTime = Optional.empty();
                    // Use drive input speeds (none if in auto)
                    return driveInputSpeeds;
                  }

                  HolonomicDriveState currentState =
                      new HolonomicDriveState(
                          robot,
                          fieldVelocity.vxMetersPerSecond,
                          fieldVelocity.vyMetersPerSecond,
                          fieldVelocity.omegaRadiansPerSecond);
                  // Sample and flip state
                  HolonomicDriveState setpointState = trajectory.sample(sampleTime);
                  setpointState = AllianceFlipUtil.apply(setpointState);
                  // calculate trajectory speeds
                  ChassisSpeeds trajectorySpeeds =
                      trajectoryController.calculate(currentState, setpointState);

                  // Log trajectory data
                  Logger.recordOutput("Trajectory/setpointPose", setpointState.pose());
                  Logger.recordOutput(
                      "Trajectory/translationError",
                      trajectoryController.getPoseError().getTranslation().getNorm());
                  Logger.recordOutput(
                      "Trajectory/rotationError",
                      trajectoryController.getPoseError().getRotation().getRadians());
                  return trajectorySpeeds;
                })
            .orElse(driveInputSpeeds); // Otherwise use inputted speeds

    // Use heading controller if there
    headingSupplier.ifPresent(
        headingSupplier -> {
          Rotation2d setpointHeading = headingSupplier.get();
          speeds.omegaRadiansPerSecond =
              headingController.calculate(
                  robot.getRotation().getRadians(), setpointHeading.getRadians());
          Logger.recordOutput("Drive/headingControl", setpointHeading);
          Logger.recordOutput(
              "Drive/headingError",
              setpointHeading.getRadians() - robot.getRotation().getRadians());
        });

    Logger.recordOutput(
        "Drive/speeds",
        new double[] {
          speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
        });
    return speeds;
  }

  public Optional<Trajectory> getCurrentTrajectory() {
    return currentTrajectory;
  }

  @AutoLogOutput(key = "Drive/followingTrajectory")
  public boolean followingTrajectory() {
    return currentTrajectory.isPresent();
  }

  @AutoLogOutput(key = "Drive/headingControlled")
  public boolean isHeadingControlled() {
    return headingSupplier.isPresent();
  }

  @AutoLogOutput(key = "Drive/xModeEnabled")
  public boolean xModeEnabled() {
    return xEnabled;
  }

  private void configTrajectoryTolerances() {
    trajectoryController.setControllerTolerance(
        new Pose2d(
            new Translation2d(
                trajectoryLinearTolerance.get(), Rotation2d.fromRadians(Math.PI / 4.0)),
            Rotation2d.fromRadians(trajectoryThetaTolerance.get())));
    double velocityXTolerance = trajectoryLinearVelocityTolerance.get() / Math.sqrt(2.0);
    trajectoryController.setGoalTolerance(
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
