package frc.robot.subsystems.superstructure.Arm;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          reduction,
          1.06328,
          armLength,
          0.0,
          Units.degreesToRadians(110.0),
          true,
          Units.degreesToRadians(0.0));

  private final ProfiledPIDController profiledController;
  private ArmFeedforward ff;
  private double appliedVoltage = 0.0;
  private boolean usingVoltageControl = false;
  private boolean homed = false;

  public ArmIOSim() {
    ff =
        new ArmFeedforward(
            controllerConstants.ffkS(),
            controllerConstants.ffkG(),
            controllerConstants.ffkV(),
            controllerConstants.ffkA());
    profiledController =
        new ProfiledPIDController(
            controllerConstants.kP(),
            controllerConstants.kI(),
            controllerConstants.kD(),
            new TrapezoidProfile.Constraints(
                profileConstraints.cruiseVelocityRadPerSec(),
                profileConstraints.accelerationRadPerSec2()));
    sim.setState(Units.degreesToRadians(45.0), 0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.update(0.02);

    inputs.armAnglePosition = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.armReferencePosition = Rotation2d.fromRadians(profiledController.getGoal().position);
    inputs.armVelocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = new double[] {appliedVoltage};
    inputs.armCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.armTorqueCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.armTempCelcius = new double[] {0.0};

    inputs.homed = homed;

    // control
    if (!usingVoltageControl) {
      appliedVoltage =
          profiledController.calculate(sim.getAngleRads())
              + ff.calculate(
                  profiledController.getSetpoint().position,
                  profiledController.getSetpoint().velocity);
      System.out.println(
          "Open loop voltage: "
              + ff.calculate(
                  profiledController.getSetpoint().position,
                  profiledController.getSetpoint().velocity));
      appliedVoltage = MathUtil.clamp(appliedVoltage, -12.0, 12.0);
      sim.setInputVoltage(appliedVoltage);
    }
  }

  @Override
  public void setSetpoint(double setpointRads) {
    usingVoltageControl = false;
    profiledController.setGoal(setpointRads);
  }

  @Override
  public void setVoltage(double volts) {
    usingVoltageControl = true;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPID(double p, double i, double d) {
    profiledController.setPID(p, i, d);
  }

  @Override
  public void setFF(double s, double v, double a, double g) {
    ff = new ArmFeedforward(s, g, v, a);
  }

  @Override
  public void setProfileConstraints(
      double cruiseVelocityRadsPerSec, double accelerationRadsPerSec2) {
    profiledController.setConstraints(
        new TrapezoidProfile.Constraints(cruiseVelocityRadsPerSec, accelerationRadsPerSec2));
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setHome() {
    homed = true;
  }
}
