// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.frc2024.Constants;

public class GenericSlamElevatorIOSim implements GenericSlamElevatorIO {
  private final ElevatorSim sim;
  private double appliedVoltage = 0.0;
  private final PIDController currentController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);

  private final double drumRadiusMeters;

  /**
   * Creates a new GenericSlamElevator Sim implementation
   *
   * @param maxLengthMeters Position in motor radians from top to bottom of slam elevator
   */
  public GenericSlamElevatorIOSim(
      double maxLengthMeters, double reduction, double drumRadiusMeters) {
    sim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            0.5,
            drumRadiusMeters,
            0.0,
            maxLengthMeters,
            false,
            0.0);
    sim.setState(maxLengthMeters / 2.0, 0);
    this.drumRadiusMeters = drumRadiusMeters;
  }

  @Override
  public void updateInputs(GenericSlamElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.positionRads = sim.getPositionMeters() / drumRadiusMeters;
    inputs.velocityRadsPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void runCurrent(double amps) {
    appliedVoltage = currentController.calculate(sim.getCurrentDrawAmps(), amps);
    appliedVoltage = MathUtil.clamp(appliedVoltage, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }
}
