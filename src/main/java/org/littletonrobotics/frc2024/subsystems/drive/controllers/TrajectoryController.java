package org.littletonrobotics.frc2024.subsystems.drive.controllers;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.trajectoryConstants;
import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.frc2024.util.trajectory.HolonomicDriveController;
import org.littletonrobotics.frc2024.util.trajectory.HolonomicTrajectory;
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

    // Run trajectory
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
    double sampletime = Timer.getFPGATimestamp() - startTime;
    VehicleState currentState =
        VehicleState.newBuilder()
            .setX(currentPose.getTranslation().getX())
            .setY(currentPose.getTranslation().getY())
            .setTheta(currentPose.getRotation().getRadians())
            .setVx(fieldVelocity.dx)
            .setVy(fieldVelocity.dy)
            .setOmega(fieldVelocity.dtheta)
            .build();
    // Sample and flip state
    VehicleState setpointState = trajectory.sample(sampletime);
    setpointState = AllianceFlipUtil.apply(setpointState);

    // calculate trajectory speeds
    ChassisSpeeds speeds = controller.calculate(currentState, setpointState);
    // Log trajectory data
    Logger.recordOutput(
        "Trajectory/SetpointPose",
        new Pose2d(
            setpointState.getX(), setpointState.getY(), new Rotation2d(setpointState.getTheta())));
    Logger.recordOutput("Trajectory/SetpointSpeeds/vx", setpointState.getVx());
    Logger.recordOutput("Trajectory/SetpointSpeeds/vy", setpointState.getVy());
    Logger.recordOutput("Trajectory/SetpointSpeeds/omega", setpointState.getOmega());
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
    return Timer.getFPGATimestamp() - startTime > trajectory.getDuration();
  }
}
