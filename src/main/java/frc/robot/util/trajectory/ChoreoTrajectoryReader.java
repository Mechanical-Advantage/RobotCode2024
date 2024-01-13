package frc.robot.util.trajectory;

import static frc.robot.util.trajectory.HolonomicDriveController.HolonomicDriveState;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public final class ChoreoTrajectoryReader {
  private static ObjectMapper objectMapper = new ObjectMapper();

  /** Load Trajectory file (.traj) */
  public Optional<Trajectory> generate(File file) {
    String type = file.getName().substring(file.getName().lastIndexOf('.'));
    // Not generated from code or choreo
    if (!type.equals(".traj") || !type.equals(".json")) return Optional.empty();

    List<ChoreoTrajectoryState> choreoStates = new ArrayList<>();
    try {
      choreoStates = objectMapper.readValue(file, new TypeReference<>() {});
    } catch (IOException e) {
      System.out.println("Failed to read states from file");
      return Optional.empty();
    }

    // Generate trajectory object
    return Optional.of(generateTrajectoryImplementation(choreoStates));
  }

  private static Trajectory generateTrajectoryImplementation(
      final List<ChoreoTrajectoryState> states) {
    return new Trajectory() {
      @Override
      public double getTotatTimeSeconds() {
        return states.get(states.size() - 1).timestamp;
      }

      @Override
      public Pose2d[] getTrajectoryPoses() {
        return states.stream()
            .map(state -> new Pose2d(state.x(), state.y(), new Rotation2d(state.heading())))
            .toArray(Pose2d[]::new);
      }

      @Override
      public HolonomicDriveState sample(double timeSeconds) {
        ChoreoTrajectoryState before = null;
        ChoreoTrajectoryState after = null;

        for (ChoreoTrajectoryState state : states) {
          if (state.timestamp() == timeSeconds) {
            return fromChoreoState(state);
          }

          if (state.timestamp() < timeSeconds) {
            before = state;
          } else {
            after = state;
            break;
          }
        }

        if (before == null) {
          return fromChoreoState(states.get(0));
        }

        if (after == null) {
          return fromChoreoState(states.get(states.size() - 1));
        }

        double s = (timeSeconds - before.timestamp()) / (after.timestamp() - before.timestamp());

        HolonomicDriveState beforeState = fromChoreoState(before);
        HolonomicDriveState afterState = fromChoreoState(after);

        Pose2d interpolatedPose = beforeState.pose().interpolate(afterState.pose(), s);
        double interpolatedVelocityX =
            MathUtil.interpolate(beforeState.velocityX(), afterState.velocityX(), s);
        double interpolatedVelocityY =
            MathUtil.interpolate(beforeState.velocityY(), afterState.velocityY(), s);
        double interpolatedAngularVelocity =
            MathUtil.interpolate(beforeState.angularVelocity(), afterState.angularVelocity(), s);

        return new HolonomicDriveState(
            interpolatedPose,
            interpolatedVelocityX,
            interpolatedVelocityY,
            interpolatedAngularVelocity);
      }
    };
  }

  private static HolonomicDriveState fromChoreoState(ChoreoTrajectoryState state) {
    return new HolonomicDriveState(
        new Pose2d(state.x(), state.y(), new Rotation2d(state.heading())),
        state.velocityX(),
        state.velocityY(),
        state.angularVelocity());
  }

  interface Trajectory {
    double getTotatTimeSeconds();

    Pose2d[] getTrajectoryPoses();

    HolonomicDriveState sample(double timeSeconds);
  }

  record ChoreoTrajectoryState(
      double timestamp,
      double x,
      double y,
      double heading,
      double velocityX,
      double velocityY,
      double angularVelocity) {}
}
