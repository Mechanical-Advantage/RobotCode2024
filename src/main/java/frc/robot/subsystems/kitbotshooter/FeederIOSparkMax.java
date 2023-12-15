package frc.robot.subsystems.kitbotshooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FeederIOSparkMax implements FeederIO {
  private static final double GEARING = (1.0 / 1.0);

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public FeederIOSparkMax() {
    motor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);

    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(40);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12.0);

    encoder = motor.getEncoder();
    encoder.setPosition(0.0);
    encoder.setAverageDepth(2);
    encoder.setMeasurementPeriod(Constants.loopPeriodMs);

    motor.burnFlash();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederPositionRads = Units.rotationsToRadians(encoder.getPosition()) / GEARING;
    inputs.feederVelocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity()) / GEARING;
    inputs.feederAppliedVolts = motor.getAppliedOutput();
    inputs.feederCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(encoder.getVelocity()) / GEARING;
  }
}
