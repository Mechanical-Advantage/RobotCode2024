package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterIOSparkMax implements ShooterIO {
  private CANSparkFlex leftMotor;
  private CANSparkFlex rightMotor;

  private CANSparkFlex feederMotor;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private SparkPIDController leftController;
  private SparkPIDController rightController;
  private SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public ShooterIOSparkMax() {
    leftMotor = new CANSparkFlex(leftFlywheelConstants.id(), CANSparkFlex.MotorType.kBrushless);
    rightMotor = new CANSparkFlex(rightFlywheelConstants.id(), CANSparkFlex.MotorType.kBrushless);
    feederMotor = new CANSparkFlex(feederConstants.id(), CANSparkFlex.MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    feederMotor.restoreFactoryDefaults();

    leftMotor.setInverted(leftFlywheelConstants.inverted());
    rightMotor.setInverted(rightFlywheelConstants.inverted());
    leftMotor.setSmartCurrentLimit(60);
    rightMotor.setSmartCurrentLimit(60);
    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);

    feederMotor.setInverted(feederConstants.inverted());
    feederMotor.setSmartCurrentLimit(100);
    feederMotor.enableVoltageCompensation(12.0);

    setShooterBrakeMode(false);
    setFeederBrakeMode(false);

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
    //    leftEncoder.setMeasurementPeriod(10);
    //    rightEncoder.setMeasurementPeriod(10);
    //    leftEncoder.setAverageDepth(2);
    //    rightEncoder.setAverageDepth(2);

    // rotations, rps
    leftEncoder.setPositionConversionFactor(1.0 / flywheelReduction);
    rightEncoder.setPositionConversionFactor(1.0 / flywheelReduction);
    leftEncoder.setVelocityConversionFactor(1.0 / flywheelReduction);
    rightEncoder.setVelocityConversionFactor(1.0 / flywheelReduction);

    leftController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();
    leftController.setFeedbackDevice(leftEncoder);
    rightController.setFeedbackDevice(rightEncoder);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
    feederMotor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftFlywheelPositionRotations = leftEncoder.getPosition();
    inputs.leftFlywheelVelocityRPM = leftEncoder.getVelocity();
    inputs.leftFlywheelAppliedVolts = leftMotor.getAppliedOutput();
    inputs.leftFlywheelOutputCurrent = leftMotor.getOutputCurrent();

    inputs.rightFlywheelPositionRotations = rightEncoder.getPosition();
    inputs.rightFlywheelVelocityRPM = rightEncoder.getVelocity();
    inputs.rightFlywheelAppliedVolts = rightMotor.getAppliedOutput();
    inputs.rightFlywheelOutputCurrent = rightMotor.getOutputCurrent();

    inputs.feederVelocityRPM = feederMotor.getEncoder().getVelocity();
    inputs.feederAppliedVolts = feederMotor.getAppliedOutput();
    inputs.feederOutputCurrent = feederMotor.getOutputCurrent();
  }

  @Override
  public void setLeftRPM(double rpm) {
    leftController.setReference(
        rpm,
        CANSparkBase.ControlType.kVelocity,
        0,
        leftFF.calculate(rpm),
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setRightRPM(double rpm) {
    rightController.setReference(
        rpm,
        CANSparkBase.ControlType.kVelocity,
        0,
        rightFF.calculate(rpm),
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }

  @Override
  public void setLeftCharacterizationVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setRightCharacterizationVoltage(double volts) {
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setLeftBrakeMode(boolean enabled) {
    leftMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public void setRightBrakeMode(boolean enabled) {
    rightMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public void setFeederBrakeMode(boolean enabled) {
    feederMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public void setLeftPID(double p, double i, double d) {
    leftController.setP(p);
    leftController.setI(i);
    leftController.setD(d);
  }

  @Override
  public void setLeftFF(double kS, double kV, double kA) {
    leftFF = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setRightPID(double p, double i, double d) {
    rightController.setP(p);
    rightController.setI(i);
    rightController.setD(d);
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
