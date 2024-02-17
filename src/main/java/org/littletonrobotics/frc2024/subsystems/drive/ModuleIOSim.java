// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), moduleConstants.driveReduction(), 0.025);
  private final DCMotorSim turnSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), moduleConstants.turnReduction(), 0.004);

  private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0, 0.02);
  private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0, 0.02);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final Rotation2d turnAbsoluteInitPosition;

  public ModuleIOSim(ModuleConfig config) {
    turnAbsoluteInitPosition = config.absoluteEncoderOffset();
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryDrivePositionsMeters =
        new double[] {driveSim.getAngularPositionRad() * driveConfig.wheelRadius()};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRadians(turnSim.getAngularPositionRad())};
  }

  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
    setDriveVoltage(
        driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadsPerSec)
            + ffVolts);
  }

  @Override
  public void setTurnPositionSetpoint(double angleRads) {
    setTurnVoltage(turnFeedback.calculate(turnSim.getAngularPositionRad(), angleRads));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {
    driveSim.setInputVoltage(0.0);
    turnSim.setInputVoltage(0.0);
  }
}
