// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.driveConfig;
import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.moduleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.Getter;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP", moduleConstants.drivekP());
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD", moduleConstants.drivekD());
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS", moduleConstants.ffkS());
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV", moduleConstants.ffkV());
  private static final LoggedTunableNumber turnkP =
      new LoggedTunableNumber("Drive/Module/TurnkP", moduleConstants.turnkP());
  private static final LoggedTunableNumber turnkD =
      new LoggedTunableNumber("Drive/Module/TurnkD", moduleConstants.turnkD());
  private static final String[] moduleNames = new String[] {"FL", "FR", "BL", "BR"};

  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(moduleConstants.ffkS(), moduleConstants.ffkV(), 0.0);
  @Getter private SwerveModuleState setpointState = new SwerveModuleState();

  // Alerts
  private final Alert driveMotorDisconnected;
  private final Alert turnMotorDisconnected;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    driveMotorDisconnected =
        new Alert(
            "Drive", moduleNames[index] + " drive motor disconnected!", Alert.AlertType.WARNING);
    turnMotorDisconnected =
        new Alert(
            "Drive", moduleNames[index] + " turn motor disconnected!", Alert.AlertType.WARNING);
  }

  /** Called while blocking odometry thread */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + moduleNames[index] + "_Module", inputs);

    // Update ff and controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(), 0),
        drivekS,
        drivekV);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP, drivekD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setTurnPID(turnkP.get(), 0, turnkD.get()), turnkP, turnkD);

    // Display warnings
    driveMotorDisconnected.set(!inputs.driveMotorConnected);
    turnMotorDisconnected.set(!inputs.turnMotorConnected);
  }

  /** Runs to {@link SwerveModuleState} */
  public void runSetpoint(SwerveModuleState setpoint) {
    setpointState = setpoint;
    io.runDriveVelocitySetpoint(
        setpoint.speedMetersPerSecond / driveConfig.wheelRadius(),
        ff.calculate(setpoint.speedMetersPerSecond / driveConfig.wheelRadius()));
    io.runTurnPositionSetpoint(setpoint.angle.getRadians());
  }

  /** Runs characterization volts or amps depending on using voltage or current control. */
  public void runCharacterization(double input) {
    io.runTurnPositionSetpoint(0.0);
    if (inputs.hasCurrentControl) {
      io.runDriveCurrent(input);
    } else {
      io.runDriveVolts(input);
    }
  }

  /** Sets brake mode to {@code enabled}. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Stops motors. */
  public void stop() {
    io.stop();
  }

  /** Get all latest {@link SwerveModulePosition}'s from last cycle. */
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

  /** Get turn angle of module as {@link Rotation2d}. */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Get position of wheel in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * driveConfig.wheelRadius();
  }

  /** Get velocity of wheel in m/s. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * driveConfig.wheelRadius();
  }

  /** Get current {@link SwerveModulePosition} of module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Get current {@link SwerveModuleState} of module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Get velocity of drive wheel for characterization */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
