// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getNEO(1), moduleConstants.driveReduction(), 0.025);
  private final DCMotorSim turnSim =
      new DCMotorSim(DCMotor.getNEO(1), moduleConstants.turnReduction(), 0.004);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final Rotation2d turnAbsoluteInitPosition;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  private Rotation2d turnAbsolutePosition;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop

  public ModuleIOSim(ModuleConfig config) {
    turnAbsoluteInitPosition = config.absoluteEncoderOffset();
    turnAbsolutePosition = turnAbsoluteInitPosition;

    driveFeedforward = new SimpleMotorFeedforward(moduleConstants.ffKs(), moduleConstants.ffKv());
    driveFeedback = new PIDController(moduleConstants.driveKp(), 0.0, moduleConstants.drivekD());
    turnFeedback = new PIDController(moduleConstants.turnKp(), 0.0, moduleConstants.turnkD());
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    turnAbsolutePosition = inputs.turnAbsolutePosition;
    inputs.turnPosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};

    inputs.odometryDrivePositionsMeters =
        new double[] {driveSim.getAngularPositionRad() * wheelRadius};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRadians(turnSim.getAngularPositionRad())};
  }

  @Override
  public void periodic() {
    // Run closed loop turn control
    if (angleSetpoint != null) {
      turnAppliedVolts =
          turnFeedback.calculate(turnAbsolutePosition.getRadians(), angleSetpoint.getRadians());
      turnSim.setInputVoltage(turnAppliedVolts);
      // Run closed loop drive control
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());
        double velocityRadPerSec = adjustSpeedSetpoint / wheelRadius;
        driveAppliedVolts =
            driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback.calculate(
                    driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec);
        driveSim.setInputVoltage(driveAppliedVolts);
      }
    }
  }

  @Override
  public void runSetpoint(SwerveModuleState state) {
    angleSetpoint = state.angle;
    speedSetpoint = state.speedMetersPerSecond;
  }

  @Override
  public void runCharacterization(double volts) {
    // Lock wheels to forward and no speed setpoint
    angleSetpoint = new Rotation2d();
    speedSetpoint = null;

    driveAppliedVolts = volts;
    driveSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    driveSim.setInputVoltage(0.0);
    turnSim.setInputVoltage(0.0);

    speedSetpoint = null;
    angleSetpoint = null;
  }
}
