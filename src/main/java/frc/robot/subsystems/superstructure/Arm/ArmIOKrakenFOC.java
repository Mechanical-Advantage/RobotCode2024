package frc.robot.subsystems.superstructure.Arm;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ArmIOKrakenFOC implements ArmIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final StatusSignal<Double> armPositionRotations;
  private final StatusSignal<Double> armTrajectorySetpointPositionRotations;
  private final StatusSignal<Double> armVelocityRPS;
  private final StatusSignal<Double>[] armAppliedVoltage = new StatusSignal[2];
  private final StatusSignal<Double>[] armOutputCurrent = new StatusSignal[2];
  private final StatusSignal<Double>[] armTorqueCurrent = new StatusSignal[2];
  private final StatusSignal<Double>[] armTempCelsius = new StatusSignal[2];

  Slot0Configs controllerConfig;

  public ArmIOKrakenFOC() {
    leaderMotor = new TalonFX(leaderID, "canivore");
    followerMotor = new TalonFX(followerID, "canivore");
    followerMotor.setControl(new Follower(leaderID, true));

    // Leader motor configs
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.MotorOutput.Inverted =
        leaderInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    leaderConfig.Feedback.SensorToMechanismRatio = reduction;
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
            .withMotionMagicAcceleration(
                Units.radiansToRotations(profileConstraints.accelerationRadPerSec2()));
    leaderMotor.getConfigurator().apply(leaderConfig, 1);

    // Follower configs
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Status signals
    armPositionRotations = leaderMotor.getPosition();
    armTrajectorySetpointPositionRotations = leaderMotor.getClosedLoopReference();
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
        100,
        armPositionRotations,
        armTrajectorySetpointPositionRotations,
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
    inputs.hasFoc = true;

    BaseStatusSignal.refreshAll(
        armPositionRotations,
        armTrajectorySetpointPositionRotations,
        armVelocityRPS,
        armAppliedVoltage[0],
        armAppliedVoltage[1],
        armOutputCurrent[0],
        armOutputCurrent[1],
        armTorqueCurrent[0],
        armTorqueCurrent[1],
        armTempCelsius[0],
        armTempCelsius[1]);

    inputs.armAnglePositionRads = Units.rotationsToRadians(armPositionRotations.getValue());
    inputs.armTrajectorySetpointRads =
        Units.rotationsToRadians(armTrajectorySetpointPositionRotations.getValue());
    inputs.armVelocityRadsPerSec = Units.rotationsToRadians(armVelocityRPS.getValue());
    inputs.armAppliedVolts =
        new double[] {armAppliedVoltage[0].getValueAsDouble(), armAppliedVoltage[1].getValueAsDouble()};
    inputs.armCurrentAmps =
        new double[] {armOutputCurrent[0].getValueAsDouble(), armOutputCurrent[1].getValueAsDouble()};
    inputs.armTorqueCurrentAmps =
        new double[] {armTorqueCurrent[0].getValueAsDouble(), armTorqueCurrent[1].getValueAsDouble()};
    inputs.armTempCelcius =
        new double[] {armTempCelsius[0].getValueAsDouble(), armTempCelsius[1].getValueAsDouble()};
  }

  @Override
  public void setSetpoint(double setpointRads) {
    leaderMotor.setControl(new MotionMagicTorqueCurrentFOC(Units.radiansToRotations(setpointRads)));
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(new VoltageOut(volts).withEnableFOC(true));
  }

  @Override
  public void setCurrent(double amps) {
    leaderMotor.setControl(new TorqueCurrentFOC(amps));
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
  public void setPosition(double positionRads) {
    leaderMotor.setPosition(Units.radiansToRotations(positionRads));
  }

  @Override
  public void stop() {
    leaderMotor.setControl(new NeutralOut());
  }
}
