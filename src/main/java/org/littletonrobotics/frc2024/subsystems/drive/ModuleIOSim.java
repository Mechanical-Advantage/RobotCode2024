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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.frc2024.Constants;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), moduleConstants.driveReduction(), 0.025);
  private final DCMotorSim turnSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), moduleConstants.turnReduction(), 0.004);

  private final PIDController driveFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final PIDController turnFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final Rotation2d turnAbsoluteInitPosition;

  public ModuleIOSim(ModuleConfig config) {
    turnAbsoluteInitPosition = config.absoluteEncoderOffset();
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    driveSim.update(Constants.loopPeriodSecs);
    turnSim.update(Constants.loopPeriodSecs);

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

  public void runDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  public void runTurnVolts(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    runDriveVolts(
        driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadsPerSec)
            + feedForward);
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    runTurnVolts(turnFeedback.calculate(turnSim.getAngularPositionRad(), angleRads));
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
    runDriveVolts(0.0);
    runTurnVolts(0.0);
  }
}
