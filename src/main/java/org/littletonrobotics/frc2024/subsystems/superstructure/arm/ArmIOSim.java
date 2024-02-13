package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import static org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(2),
          reduction,
          1.06328,
          armLength,
          minAngle.getRadians(),
          maxAngle.getRadians(),
          true,
          Units.degreesToRadians(0.0));

  private final ProfiledPIDController profiledController;
  private ArmFeedforward ff;
  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = false;

  public ArmIOSim() {
    ff = new ArmFeedforward(gains.ffkS(), gains.ffkG(), gains.ffkV(), gains.ffkA());
    profiledController =
        new ProfiledPIDController(
            gains.kP(),
            gains.kI(),
            gains.kD(),
            new TrapezoidProfile.Constraints(
                profileConstraints.cruiseVelocityRadPerSec(),
                profileConstraints.accelerationRadPerSec2()),
            0.001);
    sim.setState(Units.degreesToRadians(45.0), 0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    if (controllerNeedsReset) {
      profiledController.reset(sim.getAngleRads() + positionOffset, sim.getVelocityRadPerSec());
      controllerNeedsReset = false;
    }

    for (int i = 0; i < 20; i++) {
      // control
      if (closedLoop) {
        appliedVoltage =
            profiledController.calculate(sim.getAngleRads() + positionOffset)
                + ff.calculate(
                    profiledController.getSetpoint().position,
                    profiledController.getSetpoint().velocity);
        sim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
      }
      sim.update(0.001);
    }

    inputs.armPositionRads = sim.getAngleRads() + positionOffset;
    inputs.armTrajectorySetpointRads = profiledController.getSetpoint().position;
    inputs.armVelocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = new double[] {appliedVoltage};
    inputs.armCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.armTorqueCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.armTempCelcius = new double[] {0.0};
  }

  @Override
  public void setSetpoint(double setpointRads) {
    if (!closedLoop) {
      controllerNeedsReset = true;
    }
    closedLoop = true;
    profiledController.setGoal(setpointRads);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
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
  public void setPosition(double position) {
    positionOffset = position - sim.getAngleRads();
  }
}
