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

package frc.robot.subsystems.flywheel;

import static com.ctre.phoenix.motorcontrol.NeutralMode.Coast;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonSRX implements FlywheelIO {
  private static final double GEAR_RATIO = 1;
  private static final double MAX_VOLTAGE = 12;

  private TalonSRX leader;
  private double leaderPosition;
  private double leaderVelocity;
  private double leaderAppliedVolts;
  private double leaderCurrent;
  private double shooterPercent;

  public FlywheelIOTalonSRX(int controllerNum) {
    leader = new TalonSRX(controllerNum);
    leaderPosition = leader.getSelectedSensorPosition();
    leaderVelocity = leader.getSelectedSensorVelocity();
    leaderAppliedVolts = leader.getMotorOutputVoltage();
    leaderCurrent = leader.getStatorCurrent();
    shooterPercent = 0.0;
    var config = new TalonSRXConfiguration();

    config.peakCurrentLimit = 80;
    config.peakCurrentDuration = 100;
    config.continuousCurrentLimit = 60;

    leader.setNeutralMode(Coast);

    leader.configAllSettings(config);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    leaderPosition = leader.getSelectedSensorPosition();
    leaderVelocity = leader.getSelectedSensorVelocity();
    leaderAppliedVolts = leader.getMotorOutputVoltage();
    leaderCurrent = leader.getStatorCurrent();

    inputs.positionRad = Units.rotationsToRadians(leaderPosition) / GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(leaderVelocity) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts;
    inputs.currentAmps = new double[] {leaderCurrent, 0};
    inputs.shooterPercent = shooterPercent;
  }

  @Override
  public void setVoltage(double volts) {
    leader.set(TalonSRXControlMode.PercentOutput, volts / MAX_VOLTAGE);
    shooterPercent = volts / MAX_VOLTAGE;
  }

  @Override
  public void stop() {
    leader.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var slot = new SlotConfiguration();
    slot.kP = kP;
    slot.kI = kI;
    slot.kD = kD;
    leader.configureSlot(slot);
  }
}
