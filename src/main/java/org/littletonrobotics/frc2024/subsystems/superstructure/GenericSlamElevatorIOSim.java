// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.frc2024.Constants;

public class GenericSlamElevatorIOSim implements GenericSlamElevatorIO {
  ElevatorSim sim;
  private double appliedVoltage = 0.0;

  public GenericSlamElevatorIOSim(DCMotor motorModel, double maxLength) {
    sim = new ElevatorSim(motorModel, 1.0, 1.0, 1.0, 0.0, maxLength, false, 0.0);
  }

  @Override
  public void updateInputs(GenericSlamElevatorIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);
    inputs.positionRads = sim.getPositionMeters(); // Radius of 1
    inputs.velocityRadsPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
