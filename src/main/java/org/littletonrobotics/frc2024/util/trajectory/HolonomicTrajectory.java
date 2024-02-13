// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.util.trajectory;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Path;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleState;

public class HolonomicTrajectory {
  private final Trajectory trajectory;

  public HolonomicTrajectory(String name) {
    File file =
        Path.of(Filesystem.getDeployDirectory().getPath(), "trajectories", name + ".pathblob")
            .toFile();
    try {
      InputStream fileStream = new FileInputStream(file);
      trajectory = Trajectory.parseFrom(fileStream);
    } catch (IOException e) {
      System.err.println("Could not load trajectory \"" + name + "\"");
      throw new RuntimeException();
    }
  }

  public double getDuration() {
    if (trajectory.getStatesCount() > 0) {
      return trajectory.getStates(trajectory.getStatesCount() - 1).getTime();
    } else {
      return 0.0;
    }
  }

  public Pose2d getStartPose() {
    VehicleState startState = getStartState();
    return new Pose2d(startState.getX(), startState.getY(), new Rotation2d(startState.getTheta()));
  }

  public Pose2d[] getTrajectoryPoses() {
    Pose2d[] poses = new Pose2d[trajectory.getStatesCount()];

    for (int i = 0; i < trajectory.getStatesCount(); i++) {
      VehicleState state = trajectory.getStates(i).getState();
      poses[i] = new Pose2d(state.getX(), state.getY(), new Rotation2d(state.getTheta()));
    }
    return poses;
  }

  public VehicleState getStartState() {
    return trajectory.getStates(0).getState();
  }

  public VehicleState getEndState() {
    return trajectory.getStates(trajectory.getStatesCount() - 1).getState();
  }

  public VehicleState sample(double timeSeconds) {
    TimestampedVehicleState before = null;
    TimestampedVehicleState after = null;

    for (TimestampedVehicleState state : trajectory.getStatesList()) {
      if (state.getTime() == timeSeconds) {
        return state.getState();
      }

      if (state.getTime() < timeSeconds) {
        before = state;
      } else {
        after = state;
        break;
      }
    }

    if (before == null) {
      return trajectory.getStates(0).getState();
    }

    if (after == null) {
      return trajectory.getStates(trajectory.getStatesCount() - 1).getState();
    }

    double s = (timeSeconds - before.getTime()) / (after.getTime() - before.getTime());

    Pose2d beforePose =
        new Pose2d(
            before.getState().getX(),
            before.getState().getY(),
            new Rotation2d(before.getState().getTheta()));
    Pose2d afterPose =
        new Pose2d(
            after.getState().getX(),
            after.getState().getY(),
            new Rotation2d(after.getState().getTheta()));

    Pose2d interpolatedPose = beforePose.interpolate(afterPose, s);
    double interpolatedVelocityX =
        MathUtil.interpolate(before.getState().getVx(), after.getState().getVx(), s);
    double interpolatedVelocityY =
        MathUtil.interpolate(before.getState().getVy(), after.getState().getVy(), s);
    double interpolatedAngularVelocity =
        MathUtil.interpolate(before.getState().getOmega(), after.getState().getOmega(), s);

    return VehicleState.newBuilder()
        .setX(interpolatedPose.getTranslation().getX())
        .setY(interpolatedPose.getTranslation().getY())
        .setTheta(interpolatedPose.getRotation().getRadians())
        .setVx(interpolatedVelocityX)
        .setVy(interpolatedVelocityY)
        .setOmega(interpolatedAngularVelocity)
        .build();
  }
}
