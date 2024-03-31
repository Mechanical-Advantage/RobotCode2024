// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;

/** Generic roller IO implementation for a roller or series of rollers using a SPARK Flex. */
public abstract class GenericRollerSystemIOSpark implements GenericRollerSystemIO {
  private final CANSparkBase motor;
  private final RelativeEncoder encoder;

  private final double reduction;

  public GenericRollerSystemIOSpark(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction,
      boolean isFlex) {
    this.reduction = reduction;

    if (isFlex) {
      motor = new CANSparkFlex(id, CANSparkBase.MotorType.kBrushless);
    } else {
      motor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    }

    motor.setSmartCurrentLimit(currentLimitAmps);
    motor.setInverted(invert);
    motor.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    encoder = motor.getEncoder();
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
