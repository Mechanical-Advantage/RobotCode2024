package org.littletonrobotics.frc2024.subsystems.flywheels;

import static org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class FlywheelsIOSparkFlex implements FlywheelsIO {
  // Hardware
  private final CANSparkFlex leftMotor;
  private final CANSparkFlex rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // Controllers
  private final SparkPIDController leftController;
  private final SparkPIDController rightController;
  // Open loop
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public FlywheelsIOSparkFlex() {
    // Init Hardware
    leftMotor = new CANSparkFlex(config.leftID(), CANSparkFlex.MotorType.kBrushless);
    rightMotor = new CANSparkFlex(config.rightID(), CANSparkFlex.MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    // Config Hardware
    // Default
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    // Limits
    leftMotor.setSmartCurrentLimit(60);
    rightMotor.setSmartCurrentLimit(60);
    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);

    // Reset encoders
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
    leftEncoder.setMeasurementPeriod(10);
    rightEncoder.setMeasurementPeriod(10);
    leftEncoder.setAverageDepth(2);
    rightEncoder.setAverageDepth(2);

    // Get controllers
    leftController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();
    setPID(gains.kP(), gains.kI(), gains.kD());
    setFF(gains.kS(), gains.kV(), gains.kA());

    // Disable brake mode
    leftMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    rightMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.leftPositionRads =
        Units.rotationsToRadians(leftEncoder.getPosition()) / config.reduction();
    inputs.leftVelocityRpm = leftEncoder.getVelocity() / config.reduction();
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput();
    inputs.leftOutputCurrent = leftMotor.getOutputCurrent();
    inputs.leftTempCelsius = leftMotor.getMotorTemperature();

    inputs.rightPositionRads =
        Units.rotationsToRadians(rightEncoder.getPosition()) / config.reduction();
    inputs.rightVelocityRpm = rightEncoder.getVelocity() / config.reduction();
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput();
    inputs.rightOutputCurrent = rightMotor.getOutputCurrent();
    inputs.rightTempCelsius = rightMotor.getMotorTemperature();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  @Override
  public void runVelocity(double leftRpm, double rightRpm) {
    leftController.setReference(
        leftRpm * config.reduction(),
        CANSparkBase.ControlType.kVelocity,
        0,
        ff.calculate(leftRpm),
        SparkPIDController.ArbFFUnits.kVoltage);
    rightController.setReference(
        rightRpm * config.reduction(),
        CANSparkBase.ControlType.kVelocity,
        0,
        ff.calculate(rightRpm),
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    leftController.setP(kP);
    leftController.setI(kI);
    leftController.setD(kD);
    rightController.setP(kP);
    rightController.setI(kI);
    rightController.setD(kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void runCharacterizationLeftVolts(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void runCharacterizationRightVolts(double volts) {
    rightMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
