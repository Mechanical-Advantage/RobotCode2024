// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.flywheels;

import static org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelsIOSim implements FlywheelsIO {
  private final FlywheelSim leftSim =
      new FlywheelSim(DCMotor.getKrakenX60Foc(1), config.reduction(), 0.00363458292);
  private final FlywheelSim rightSim =
      new FlywheelSim(DCMotor.getKrakenX60Foc(1), config.reduction(), 0.00363458292);

  private final PIDController leftController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());
  private final PIDController rightController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());
  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  private Double leftSetpointRPM = null;
  private Double rightSetpointRPM = null;

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    leftSim.update(0.02);
    rightSim.update(0.02);
    // control to setpoint
    if (leftSetpointRPM != null && rightSetpointRPM != null) {
      runVolts(
          leftController.calculate(leftSim.getAngularVelocityRPM(), leftSetpointRPM)
              + ff.calculate(leftSetpointRPM),
          rightController.calculate(rightSim.getAngularVelocityRPM(), rightSetpointRPM)
              + ff.calculate(rightSetpointRPM));
    }

    inputs.leftPositionRads +=
        Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.leftVelocityRpm = leftSim.getAngularVelocityRPM();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftOutputCurrent = leftSim.getCurrentDrawAmps();

    inputs.rightPositionRads +=
        Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.rightVelocityRpm = rightSim.getAngularVelocityRPM();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightOutputCurrent = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    leftSim.setInputVoltage(leftAppliedVolts);
    rightSim.setInputVoltage(rightAppliedVolts);
  }

  @Override
  public void runVelocity(double leftRpm, double rightRpm) {
    leftSetpointRPM = leftRpm;
    rightSetpointRPM = rightRpm;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    leftController.setPID(kP, kI, kD);
    rightController.setPID(kP, kI, kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void stop() {
    runVelocity(0.0, 0.0);
  }

  @Override
  public void runCharacterizationLeftVolts(double volts) {
    leftSetpointRPM = null;
    rightSetpointRPM = null;
    runVolts(volts, 0.0);
  }

  @Override
  public void runCharacterizationRightVolts(double volts) {
    leftSetpointRPM = null;
    rightSetpointRPM = null;
    runVolts(0.0, volts);
  }
}
