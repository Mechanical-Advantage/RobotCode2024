// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive.controllers;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.trajectoryConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2024.subsystems.drive.trajectory.TrajectoryGenerationHelpers;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleState;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class TrajectoryController {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("Trajectory/linearkP", trajectoryConstants.linearkP());
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("Trajectory/linearkD", trajectoryConstants.linearkD());
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("Trajectory/thetakP", trajectoryConstants.thetakP());
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("Trajectory/thetakD", trajectoryConstants.thetakD());

  private final HolonomicTrajectory trajectory;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private final Timer timer = new Timer();

  @Getter
  private List<Vector<N2>> moduleForces =
      IntStream.range(0, 4).boxed().map(i -> VecBuilder.fill(0, 0)).toList();

  public TrajectoryController(HolonomicTrajectory trajectory) {
    this.trajectory = trajectory;
    xController = new PIDController(linearkP.get(), 0, linearkD.get());
    yController = new PIDController(linearkP.get(), 0, linearkD.get());
    thetaController = new PIDController(thetakP.get(), 0, thetakD.get());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    timer.start();

    // Log poses
    Logger.recordOutput(
        "Trajectory/TrajectoryPoses",
        Arrays.stream(trajectory.getTrajectoryPoses())
            .map(AllianceFlipUtil::apply)
            .toArray(Pose2d[]::new));
  }

  public ChassisSpeeds update() {
    // Run trajectory
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
    double sampletime = timer.get();
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
    VehicleState setpointState = AllianceFlipUtil.apply(trajectory.sample(sampletime));
    RobotState.getInstance().setTrajectorySetpoint(setpointState.getPose());

    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentState.getX(), setpointState.getX());
    double yFeedback = yController.calculate(currentState.getY(), setpointState.getY());
    double thetaFeedback =
        thetaController.calculate(
            MathUtil.angleModulus(currentState.getTheta()),
            MathUtil.angleModulus(setpointState.getTheta()));

    // Return next output.
    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            setpointState.getVx() + xFeedback,
            setpointState.getVy() + yFeedback,
            setpointState.getOmega() + thetaFeedback,
            currentPose.getRotation());

    moduleForces =
        setpointState.getModuleForcesList().stream()
            .map(
                forces ->
                    new Translation2d(forces.getFx(), forces.getFy())
                        .rotateBy(Rotation2d.fromRadians(setpointState.getTheta()).unaryMinus())
                        .toVector())
            .toList();

    // Log trajectory data
    Logger.recordOutput("Trajectory/SetpointPose", setpointState.getPose());
    Logger.recordOutput("Trajectory/SetpointSpeeds/vx", setpointState.getVx());
    Logger.recordOutput("Trajectory/SetpointSpeeds/vy", setpointState.getVy());
    Logger.recordOutput("Trajectory/SetpointSpeeds/omega", setpointState.getOmega());
    Logger.recordOutput("Trajectory/OutputSpeeds", outputSpeeds);
    Logger.recordOutput(
        "Trajectory/TranslationError",
        currentState
            .getPose()
            .getTranslation()
            .getDistance(setpointState.getPose().getTranslation()));
    Logger.recordOutput(
        "Trajectory/RotationError",
        currentState.getPose().getRotation().minus(setpointState.getPose().getRotation()));
    return outputSpeeds;
  }

  @AutoLogOutput(key = "Trajectory/Finished")
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getDuration());
  }
}
