package frc.robot.subsystems.superstructure.Arm;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ArmIOKrakenFOC implements ArmIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  Double home = null;
  private final StatusSignal<Double> armPositionRotations;
  private final StatusSignal<Double> armReferencePositionRotations;
  private final StatusSignal<Double> armVelocityRPS;
  private final StatusSignal<Double>[] armAppliedVoltage = new StatusSignal[2];
  private final StatusSignal<Double>[] armOutputCurrent = new StatusSignal[2];
  private final StatusSignal<Double>[] armTorqueCurrent = new StatusSignal[2];
  private final StatusSignal<Double>[] armTempCelsius = new StatusSignal[2];

  Slot0Configs controllerConfig;

  public ArmIOKrakenFOC() {
    leaderMotor = new TalonFX(leaderID, "*");
    followerMotor = new TalonFX(followerID, "*");
    followerMotor.setControl(new Follower(leaderID, true));

    // Leader motor configs
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.Voltage.PeakForwardVoltage = 12.0;
    leaderConfig.Voltage.PeakReverseVoltage = 12.0;
    leaderConfig.MotorOutput.Inverted =
        leaderInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    leaderConfig.Feedback.RotorToSensorRatio = 1.0 / reduction;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    controllerConfig =
        new Slot0Configs()
            .withKP(controllerConstants.kP())
            .withKI(controllerConstants.kI())
            .withKD(controllerConstants.kD())
            .withKS(controllerConstants.ffkS())
            .withKV(controllerConstants.ffkV())
            .withKA(controllerConstants.ffkA())
            .withKG(controllerConstants.ffkG())
            .withGravityType(GravityTypeValue.Arm_Cosine);
    leaderConfig.Slot0 = controllerConfig;

    leaderConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(
                Units.radiansToRotations(profileConstraints.cruiseVelocityRadPerSec()))
            .withMotionMagicCruiseVelocity(
                Units.radiansToRotations(profileConstraints.accelerationRadPerSec2()));
    leaderMotor.getConfigurator().apply(leaderConfig, 1);

    // Follower configs
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Status signals
    armPositionRotations = leaderMotor.getPosition();
    armReferencePositionRotations = leaderMotor.getClosedLoopReference();
    armVelocityRPS = leaderMotor.getVelocity();
    armAppliedVoltage[0] = leaderMotor.getMotorVoltage();
    armAppliedVoltage[1] = followerMotor.getMotorVoltage();
    armOutputCurrent[0] = leaderMotor.getSupplyCurrent();
    armOutputCurrent[1] = followerMotor.getSupplyCurrent();
    armTorqueCurrent[0] = leaderMotor.getTorqueCurrent();
    armTorqueCurrent[1] = followerMotor.getTorqueCurrent();
    armTempCelsius[0] = leaderMotor.getDeviceTemp();
    armTempCelsius[1] = followerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        armPositionRotations,
        armReferencePositionRotations,
        armVelocityRPS,
        armAppliedVoltage[0],
        armAppliedVoltage[1],
        armOutputCurrent[0],
        armOutputCurrent[1],
        armTorqueCurrent[0],
        armTorqueCurrent[1],
        armTempCelsius[0],
        armTempCelsius[1]);
  }

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armPositionRotations,
        armReferencePositionRotations,
        armVelocityRPS,
        armAppliedVoltage[0],
        armAppliedVoltage[1],
        armOutputCurrent[0],
        armOutputCurrent[1],
        armTorqueCurrent[0],
        armTorqueCurrent[1],
        armTempCelsius[0],
        armTempCelsius[1]);

    inputs.armAnglePosition = Rotation2d.fromRotations(armPositionRotations.getValue() - home);
    inputs.armReferencePosition =
        Rotation2d.fromRotations(armReferencePositionRotations.getValue() - home);
    inputs.armVelocityRadsPerSec = Units.rotationsToRadians(armVelocityRPS.getValue());
    inputs.armAppliedVolts =
        new double[] {armAppliedVoltage[0].getValue(), armAppliedVoltage[1].getValue()};
    inputs.armCurrentAmps =
        new double[] {armOutputCurrent[0].getValue(), armOutputCurrent[1].getValue()};
    inputs.armTorqueCurrentAmps =
        new double[] {armTorqueCurrent[0].getValue(), armTorqueCurrent[1].getValue()};
    inputs.armTempCelcius =
        new double[] {armTempCelsius[0].getValue(), armTempCelsius[1].getValue()};
    inputs.homed = home != null;
  }

  @Override
  public void setSetpoint(double setpointRads) {
    leaderMotor.setControl(new MotionMagicTorqueCurrentFOC(Units.radiansToRotations(setpointRads)));
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controllerConfig.kP = p;
    controllerConfig.kI = i;
    controllerConfig.kD = d;
    leaderMotor.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void setFF(double s, double v, double a, double g) {
    controllerConfig.kS = s;
    controllerConfig.kV = v;
    controllerConfig.kA = a;
    controllerConfig.kG = g;
    leaderMotor.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void setProfileConstraints(
      double cruiseVelocityRadsPerSec, double accelerationRadsPerSec2) {
    leaderMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Units.radiansToRotations(cruiseVelocityRadsPerSec))
                .withMotionMagicAcceleration(Units.radiansToRotations(accelerationRadsPerSec2)));
  }

  @Override
  public void setHome() {
    home = armPositionRotations.getValue();
    leaderMotor.setPosition(0.0, 0.01);
  }
}
