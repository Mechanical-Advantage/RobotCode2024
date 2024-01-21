package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
  }

  /** Called while blocking odometry thread */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  public void periodic() {
    io.periodic();
  }

  public void runSetpoint(SwerveModuleState setpoint) {
    io.runSetpoint(setpoint);
  }

  public void runCharacterization(double volts) {
    io.runCharacterization(volts);
  }

  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  public void stop() {
    io.stop();
  }

  public SwerveModulePosition[] getModulePositions() {
    int minOdometryPositions =
        Math.min(inputs.odometryDrivePositionsMeters.length, inputs.odometryTurnPositions.length);
    return IntStream.range(0, minOdometryPositions)
        .mapToObj(
            odometryIndex ->
                new SwerveModulePosition(
                    inputs.odometryDrivePositionsMeters[odometryIndex],
                    inputs.odometryTurnPositions[odometryIndex]))
        .toArray(SwerveModulePosition[]::new);
  }

  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
