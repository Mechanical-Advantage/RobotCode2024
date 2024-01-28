package frc.robot.subsystems.superstructure.intake;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public IntakeIOSparkMax() {
    motor = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(inverted);
    motor.setSmartCurrentLimit(80);
    motor.enableVoltageCompensation(12.0);

    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(1.0 / reduction * 2 * Math.PI);
    encoder.setVelocityConversionFactor(1.0 / reduction * 2 * Math.PI);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadsPerSec = encoder.getVelocity();
    inputs.positionRads = encoder.getPosition();
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motor.setIdleMode(enabled ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
