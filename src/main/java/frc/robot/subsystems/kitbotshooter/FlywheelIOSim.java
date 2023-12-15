package frc.robot.subsystems.kitbotshooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import java.util.Arrays;

public class FlywheelIOSim implements FlywheelIO {
  private static final SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  private static final PIDController feedback = new PIDController(0.0, 0.0, 0.0);

  // how do I calculate that one
  private final FlywheelSim sim = new FlywheelSim(DCMotor.getNeoVortex(1), (1.0 / 1.0), 0.001);

  private double positionRads = 0.0;
  private double inputVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    positionRads += sim.getAngularVelocityRadPerSec() * (Constants.loopPeriodMs / 1000.0);
    inputs.flywheelPositionRads = new double[] {positionRads, positionRads};
    inputs.flywheelPositionRads = new double[2];
    Arrays.fill(inputs.flywheelPositionRads, sim.getAngularVelocityRadPerSec());
    inputs.flywheelAppliedVolts = new double[] {inputVolts, inputVolts};
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    double ffVolts = ffModel.calculate(velocityRadPerSec);
    double pidEffort = feedback.calculate(sim.getAngularVelocityRadPerSec(), velocityRadPerSec);
    inputVolts = MathUtil.clamp(ffVolts + pidEffort, -12.0, 12.0);
    sim.setInputVoltage(inputVolts);
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
