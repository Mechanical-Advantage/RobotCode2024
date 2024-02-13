package org.littletonrobotics.frc2024.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class GenericRollerSystemIOSim implements GenericRollerSystemIO {
  private final FlywheelSim sim;
  private double appliedVoltage = 0.0;

  public GenericRollerSystemIOSim(DCMotor motorModel, double reduction, double moi) {
    sim = new FlywheelSim(motorModel, reduction, moi);
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    sim.update(0.02);
    inputs.positionRads += sim.getAngularVelocityRadPerSec() * 0.02;
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.outputCurrent = sim.getCurrentDrawAmps();
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
