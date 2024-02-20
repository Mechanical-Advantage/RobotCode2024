// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOSparkFlex;

public class IndexerIODevbot extends GenericRollerSystemIOSparkFlex implements IndexerIO {
  private static final double reduction = (18.0 / 12.0);
  private static final int id = 27;
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = false;

  private final TalonFX middleMotor;

  public IndexerIODevbot() {
    super(id, currentLimitAmps, inverted, true, reduction);
    middleMotor = new TalonFX(7, "rio");

    TalonFXConfiguration middleConfig = new TalonFXConfiguration();
    middleConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    middleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    middleConfig.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    middleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    middleMotor.getConfigurator().apply(middleConfig);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
    middleMotor.setControl(new VoltageOut(volts).withUpdateFreqHz(0.0));
  }

  @Override
  public void stop() {
    super.stop();
    middleMotor.setControl(new NeutralOut());
  }
}
