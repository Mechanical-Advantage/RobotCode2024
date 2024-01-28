package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private final FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.01);

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sim.update(0.02);
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.positionRads += sim.getAngularVelocityRadPerSec() * 0.02;
    inputs.appliedVoltage = appliedVoltage;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(0.0);
  }
}
