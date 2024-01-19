package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  /** Odometry data from one main robot loop cycle */
  public record OdometryObservation(
      SwerveDriveWheelPositions[] wheelPositions,
      Rotation2d[] gyroAngles,
      double[] timestamps,
      boolean gyroConnected) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  private static final double poseBufferSizeSeconds = 2.0;

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
  private final Matrix<N3, N1> odometryStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroAngle = new Rotation2d();

  private RobotState() {
    odometryStdDevs.setColumn(0, DriveConstants.odometryStateStDevs.extractColumnVector(0));
    kinematics = DriveConstants.kinematics;
  }

  public void addOdometryData(OdometryObservation observation) {
    Pose2d lastOdometryPose = odometryPose;
    int minDelta = observation.wheelPositions().length;
    if (observation.gyroConnected) minDelta = Math.min(minDelta, observation.gyroAngles().length);
    for (int deltaIndex = 0; deltaIndex < minDelta; deltaIndex++) {
      //      double[] turnPositions = new double[4];
      //      double[] lastTurnPositions = new double[4];
      //      double[] turnPositionDeltas = new double[4];
      //      for (int i = 0; i < 4; i++) {
      //        turnPositions[i] =
      // observation.wheelPositions()[deltaIndex].positions[i].angle.getRadians();
      //        lastTurnPositions[i] = lastWheelPositions.positions[i].angle.getRadians();
      //        turnPositionDeltas[i] =
      //            observation
      //                .wheelPositions()[deltaIndex]
      //                .positions[i]
      //                .angle
      //                .minus(lastWheelPositions.positions[i].angle)
      //                .getRadians();
      //      }
      //      Logger.recordOutput("Odometry/TurnPositions", turnPositions);
      //      Logger.recordOutput("Odometry/LastTurnPositions", lastTurnPositions);
      //      Logger.recordOutput("Odometry/TurnPositionDeltas", turnPositionDeltas);
      Twist2d twist =
          kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions()[deltaIndex]);
      lastWheelPositions = observation.wheelPositions()[deltaIndex];
      // Set gyro angle to null if gyro not connected (sim)
      if (observation.gyroConnected()) {
        // Update dtheta for twist if gyro connected
        twist =
            new Twist2d(
                twist.dx,
                twist.dy,
                observation.gyroAngles()[deltaIndex].minus(lastGyroAngle).getRadians());
        lastGyroAngle = observation.gyroAngles()[deltaIndex];
      }
      // Add twist to odometry pose
      odometryPose = odometryPose.exp(twist);
      // Add pose to buffer at timestamp
      poseBuffer.addSample(observation.timestamps()[deltaIndex], odometryPose);

      // Calculate diff from last odom pose and add onto pose estimate
      estimatedPose = estimatedPose.exp(lastOdometryPose.log(odometryPose));
      // Set last odometry pose to current
      lastOdometryPose = odometryPose;
    }
  }

  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty())
      // exit if not there
      return;

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = odometryStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Twist2d twist = estimateAtTime.log(observation.visionPose());
    // scale twist by visionK
    var kTimesTwist = visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));
    Twist2d scaledTwist =
        new Twist2d(kTimesTwist.get(0, 0), kTimesTwist.get(1, 0), kTimesTwist.get(2, 0));

    // Recalculate current estimate by applying scaled twist to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.exp(scaledTwist).plus(sampleToOdometryTransform);
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(
      Pose2d pose, SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle) {
    estimatedPose = pose;
    poseBuffer.clear();

    odometryPose = pose;
    lastWheelPositions = wheelPositions;
    lastGyroAngle = gyroAngle;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  @AutoLogOutput(key = "Odometry/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }
}
