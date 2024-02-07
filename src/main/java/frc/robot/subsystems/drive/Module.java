package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("Drive/Module/DriveKp", DriveConstants.moduleConstants.driveKp());
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("Drive/Module/DriveKd", DriveConstants.moduleConstants.driveKd());
  private static final LoggedTunableNumber driveKs =
      new LoggedTunableNumber("Drive/Module/DriveKs", DriveConstants.moduleConstants.ffKs());
  private static final LoggedTunableNumber driveKv =
      new LoggedTunableNumber("Drive/Module/DriveKv", DriveConstants.moduleConstants.ffKv());
  private static final LoggedTunableNumber turnKp =
      new LoggedTunableNumber("Drive/Module/TurnKp", DriveConstants.moduleConstants.turnKp());
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("Drive/Module/TurnKd", DriveConstants.moduleConstants.turnKd());

  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final PIDController driveController =
      new PIDController(
          DriveConstants.moduleConstants.driveKp(), 0.0, DriveConstants.moduleConstants.driveKd());
  private final PIDController turnController =
      new PIDController(
          DriveConstants.moduleConstants.turnKp(), 0.0, DriveConstants.moduleConstants.turnKd());
  private SimpleMotorFeedforward driveFF =
      new SimpleMotorFeedforward(
          DriveConstants.moduleConstants.ffKs(), DriveConstants.moduleConstants.ffKv(), 0.0);

  @Getter private SwerveModuleState setpointState = new SwerveModuleState();

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Called while blocking odometry thread */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Update FF and controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          driveFF = new SimpleMotorFeedforward(driveKs.get(), driveKv.get(), 0);
          io.setDriveFF(driveKs.get(), driveKv.get(), 0);
        },
        driveKs,
        driveKv);
      LoggedTunableNumber.ifChanged(
          hashCode(), () -> io.setDrivePID(driveKp.get(), 0, driveKd.get()), driveKp, driveKd);
      LoggedTunableNumber.ifChanged(
          hashCode(), () -> io.setTurnPID(turnKp.get(), 0, turnKd.get()), turnKp, turnKd);
  }

  public void runSetpoint(SwerveModuleState setpoint) {
    setpointState = setpoint;
      io.setDriveVelocitySetpoint(
          setpoint.speedMetersPerSecond / DriveConstants.wheelRadius,
          driveFF.calculate(setpoint.speedMetersPerSecond / DriveConstants.wheelRadius));
      io.setTurnPositionSetpoint(setpoint.angle.getRadians());
  }

  public void runCharacterization(double volts) {
    io.setTurnVoltage(turnController.calculate(inputs.turnAbsolutePosition.getRadians(), 0));
    io.setDriveVoltage(volts);
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
    SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
    for (int i = 0; i < minOdometryPositions; i++) {
      positions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositionsMeters[i], inputs.odometryTurnPositions[i]);
    }
    return positions;
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
