package org.littletonrobotics.frc2024.subsystems.superstructure.shooter;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim leftSim =
            new FlywheelSim(DCMotor.getNeoVortex(1), flywheelReduction, 0.01);
    private final FlywheelSim rightSim =
            new FlywheelSim(DCMotor.getNeoVortex(1), flywheelReduction, 0.01);
    private final FlywheelSim feederSim = new FlywheelSim(DCMotor.getAndymarkRs775_125(1), 1.0, 0.01);

    private final PIDController leftController =
            new PIDController(
                    leftFlywheelConstants.kP(), leftFlywheelConstants.kI(), leftFlywheelConstants.kD());
    private final PIDController rightController =
            new PIDController(
                    rightFlywheelConstants.kP(), rightFlywheelConstants.kI(), rightFlywheelConstants.kD());
    private SimpleMotorFeedforward leftFF =
            new SimpleMotorFeedforward(
                    leftFlywheelConstants.kS(), leftFlywheelConstants.kV(), leftFlywheelConstants.kA());
    private SimpleMotorFeedforward rightFF =
            new SimpleMotorFeedforward(
                    rightFlywheelConstants.kS(), rightFlywheelConstants.kV(), rightFlywheelConstants.kA());

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double feederAppliedVolts = 0.0;

    private Double leftSetpointRPM = null;
    private Double rightSetpointRPM = null;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(0.02);
        rightSim.update(0.02);
        feederSim.update(0.02);
        // control to setpoint
        if (leftSetpointRPM != null) {
            leftAppliedVolts =
                    leftController.calculate(leftSim.getAngularVelocityRPM(), leftSetpointRPM)
                            + leftFF.calculate(leftSetpointRPM);
            leftSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
        }
        if (rightSetpointRPM != null) {
            rightAppliedVolts =
                    rightController.calculate(rightSim.getAngularVelocityRPM(), rightSetpointRPM)
                            + rightFF.calculate(rightSetpointRPM);
            rightSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
        }

        inputs.leftFlywheelPositionRotations +=
                Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec() * 0.02);
        inputs.leftFlywheelVelocityRPM = leftSim.getAngularVelocityRPM();
        inputs.leftFlywheelAppliedVolts = leftAppliedVolts;
        inputs.leftFlywheelOutputCurrent = leftSim.getCurrentDrawAmps();

        inputs.rightFlywheelPositionRotations +=
                Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec() * 0.02);
        inputs.rightFlywheelVelocityRPM = rightSim.getAngularVelocityRPM();
        inputs.rightFlywheelAppliedVolts = rightAppliedVolts;
        inputs.rightFlywheelOutputCurrent = rightSim.getCurrentDrawAmps();

        inputs.feederVelocityRPM = feederSim.getAngularVelocityRPM();
        inputs.feederAppliedVolts = feederAppliedVolts;
        inputs.feederOutputCurrent = feederSim.getCurrentDrawAmps();
    }

    @Override
    public void setLeftRPM(double rpm) {
        leftSetpointRPM = rpm;
    }

    @Override
    public void setRightRPM(double rpm) {
        rightSetpointRPM = rpm;
    }

    @Override
    public void setFeederVoltage(double volts) {
        feederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftSetpointRPM = null;
        leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        leftSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightSetpointRPM = null;
        rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void setLeftPID(double p, double i, double d) {
        leftController.setPID(p, i, d);
    }

    @Override
    public void setLeftFF(double s, double v, double a) {
        leftFF = new SimpleMotorFeedforward(s, v, a);
    }

    @Override
    public void setRightPID(double p, double i, double d) {
        rightController.setPID(p, i, d);
    }

    @Override
    public void setRightFF(double s, double v, double a) {
        rightFF = new SimpleMotorFeedforward(s, v, a);
    }

    @Override
    public void stop() {
        setRPM(0.0, 0.0);
        setFeederVoltage(0.0);
    }
}
