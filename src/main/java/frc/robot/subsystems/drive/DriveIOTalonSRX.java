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

import static com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder;
import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class DriveIOTalonSRX implements DriveIO {
  private static final double GEAR_RATIO = 10.0;

  private static final double MAX_VOLTAGE = 12.0;
  private static final double KP = 1.0; // TODO: MUST BE TUNED, consider using Phoenix Tuner X
  private static final double KD = 0.0; // TODO: MUST BE TUNED, consider using Phoenix Tuner X
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private final TalonSRX leftLeader = new TalonSRX(2);
  private final TalonSRX leftFollower = new TalonSRX(0);
  private final TalonSRX rightLeader = new TalonSRX(3);
  private final TalonSRX rightFollower = new TalonSRX(1);

  private double leftPosition = leftLeader.getSelectedSensorPosition();
  private double leftVelocity = leftLeader.getSelectedSensorVelocity();
  private double leftAppliedVolts = leftLeader.getMotorOutputVoltage();
  private double leftLeaderCurrent = leftLeader.getStatorCurrent();
  private double leftFollowerCurrent = leftFollower.getStatorCurrent();

  private double rightPosition = rightLeader.getSelectedSensorPosition();
  private double rightVelocity = rightLeader.getSelectedSensorVelocity();
  private double rightAppliedVolts = rightLeader.getMotorOutputVoltage();
  private double rightLeaderCurrent = rightLeader.getStatorCurrent();
  private double rightFollowerCurrent = rightFollower.getStatorCurrent();

  // private final Pigeon2 pigeon = new Pigeon2(20);

  public DriveIOTalonSRX() {
    var config = new TalonSRXConfiguration();

    config.peakCurrentLimit = 80;
    config.peakCurrentDuration = 100;
    config.continuousCurrentLimit = 60;

    leftLeader.setNeutralMode(Brake);
    leftFollower.setNeutralMode(Brake);
    rightLeader.setNeutralMode(Brake);
    rightFollower.setNeutralMode(Brake);

    SlotConfiguration slot = new SlotConfiguration();
    slot.kP = KP;
    slot.kD = KD;
    config.slot0 = slot;

    leftLeader.configAllSettings(config);
    leftFollower.configAllSettings(config);
    rightLeader.configAllSettings(config);
    rightFollower.configAllSettings(config);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.configSelectedFeedbackSensor(QuadEncoder);
    rightLeader.configSelectedFeedbackSensor(QuadEncoder);

    // pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    leftPosition = leftLeader.getSelectedSensorPosition();
    leftVelocity = leftLeader.getSelectedSensorVelocity();
    leftAppliedVolts = leftLeader.getMotorOutputVoltage();
    leftLeaderCurrent = leftLeader.getStatorCurrent();
    leftFollowerCurrent = leftFollower.getStatorCurrent();

    rightPosition = rightLeader.getSelectedSensorPosition();
    rightVelocity = rightLeader.getSelectedSensorVelocity();
    rightAppliedVolts = rightLeader.getMotorOutputVoltage();
    rightLeaderCurrent = rightLeader.getStatorCurrent();
    rightFollowerCurrent = rightFollower.getStatorCurrent();

    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition) / GEAR_RATIO;
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity) / GEAR_RATIO;
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftLeaderCurrent, leftFollowerCurrent};

    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition) / GEAR_RATIO;
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity) / GEAR_RATIO;
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {rightLeaderCurrent, rightFollowerCurrent};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftVolts / MAX_VOLTAGE);
    rightLeader.set(TalonSRXControlMode.PercentOutput, -1 * rightVolts / MAX_VOLTAGE);
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    setVoltage(
        pid.calculate(leftLeader.getSelectedSensorVelocity(), leftRadPerSec) + leftFFVolts,
        pid.calculate(rightLeader.getSelectedSensorVelocity(), rightRadPerSec) + rightFFVolts);
  }
}
