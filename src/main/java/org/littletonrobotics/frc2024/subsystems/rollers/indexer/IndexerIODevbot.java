// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.indexer;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSparkFlex;

public class IndexerIODevbot extends GenericRollerSystemIOSparkFlex implements IndexerIO {
  private static final double reduction = (18.0 / 12.0);
  private static final int id = 27;
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = false;

  private final CANSparkMax middleMotor;

  public IndexerIODevbot() {
    super(id, currentLimitAmps, inverted, true, reduction);
    middleMotor = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
    middleMotor.setInverted(false);
    middleMotor.setSmartCurrentLimit(40);
    middleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
    middleMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    super.stop();
    middleMotor.stopMotor();
  }
}
