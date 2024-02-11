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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveIOTalonSRX implements DriveIO {
  private static final double MAX_VOLTAGE = 12.0;
  private final TalonSRX leftLeader = new TalonSRX(2);
  private final TalonSRX leftFollower = new TalonSRX(0);
  private final TalonSRX rightLeader = new TalonSRX(3);
  private final TalonSRX rightFollower = new TalonSRX(1);

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

    leftLeader.configAllSettings(config);
    leftFollower.configAllSettings(config);
    rightLeader.configAllSettings(config);
    rightFollower.configAllSettings(config);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.configSelectedFeedbackSensor(QuadEncoder);
    rightLeader.configSelectedFeedbackSensor(QuadEncoder);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs, GyroIO.GyroIOInputs gyroInputs) {
    inputs.leftPositionRad =
        Units.rotationsToRadians(
            leftLeader.getSelectedSensorPosition() / Constants.TICKS_PER_REVOLUTION);
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(
            leftLeader.getSelectedSensorVelocity() / Constants.TICKS_PER_REVOLUTION * 10);
    inputs.leftAppliedVolts = leftLeader.getMotorOutputVoltage();
    inputs.leftCurrentAmps =
        new double[] {leftLeader.getStatorCurrent(), leftFollower.getStatorCurrent()};

    inputs.rightPositionRad =
        -1
            * Units.rotationsToRadians(
                rightLeader.getSelectedSensorPosition() / Constants.TICKS_PER_REVOLUTION);
    inputs.rightVelocityRadPerSec =
        -1
            * Units.rotationsToRadians(
                rightLeader.getSelectedSensorVelocity() / Constants.TICKS_PER_REVOLUTION * 10);
    inputs.rightAppliedVolts = rightLeader.getMotorOutputVoltage();
    inputs.rightCurrentAmps =
        new double[] {rightLeader.getStatorCurrent(), rightFollower.getStatorCurrent()};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftVolts / MAX_VOLTAGE);
    rightLeader.set(TalonSRXControlMode.PercentOutput, -1 * rightVolts / MAX_VOLTAGE);
  }
}