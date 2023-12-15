package frc.robot.subsystems.kitbotshooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getNeoVortex(1), (1.0 / 1.0), 0.002);

  private double positionRads;
  private double inputVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    positionRads += sim.getAngularVelocityRadPerSec() * (Constants.loopPeriodMs / 1000.0);
    inputs.feederPositionRads = positionRads;
    inputs.feederVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.feederAppliedVolts = inputVolts;
  }

  @Override
  public void runVolts(double volts) {
    inputVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(inputVolts);
  }

  @Override
  public double getVelocityRPM() {
    return sim.getAngularVelocityRPM();
  }
}
