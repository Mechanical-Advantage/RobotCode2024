package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class ArmIOKrakenFOC implements ArmIO {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final CANcoder armEncoder;

    private final StatusSignal<Double> armPositionRotations;
    private final StatusSignal<Double> absoluteEncoderPositionRotations;
    private final StatusSignal<Double> armTrajectorySetpointPositionRotations;
    private final StatusSignal<Double> armVelocityRPS;
    private final List<StatusSignal<Double>> armAppliedVoltage;
    private final List<StatusSignal<Double>> armOutputCurrent;
    private final List<StatusSignal<Double>> armTorqueCurrent;
    private final List<StatusSignal<Double>> armTempCelsius;

    Slot0Configs controllerConfig;

    public ArmIOKrakenFOC() {
        leaderMotor = new TalonFX(leaderID, "canivore");
        followerMotor = new TalonFX(followerID, "canivore");
        followerMotor.setControl(new Follower(leaderID, true));
        armEncoder = new CANcoder(armEncoderID, "canivore");

        // Arm Encoder Configs
        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
                AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
        armEncoder.getConfigurator().apply(armEncoderConfig);
        // Leader motor configs
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.MotorOutput.Inverted =
                leaderInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Feedback.FeedbackRemoteSensorID = armEncoderID;
        leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
        leaderConfig.Feedback.RotorToSensorRatio = reduction;

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
        absoluteEncoderPositionRotations = armEncoder.getPosition();
        armTrajectorySetpointPositionRotations = leaderMotor.getClosedLoopReference();
        armVelocityRPS = leaderMotor.getVelocity();
        armAppliedVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());
        armOutputCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        armTorqueCurrent = List.of(leaderMotor.getTorqueCurrent(), followerMotor.getTorqueCurrent());
        armTempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                armPositionRotations,
                absoluteEncoderPositionRotations,
                armTrajectorySetpointPositionRotations,
                armVelocityRPS,
                armAppliedVoltage.get(0),
                armAppliedVoltage.get(1),
                armOutputCurrent.get(0),
                armOutputCurrent.get(1),
                armTorqueCurrent.get(0),
                armTorqueCurrent.get(1),
                armTempCelsius.get(0),
                armTempCelsius.get(1));
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.hasFoc = true;

        BaseStatusSignal.refreshAll(
                armPositionRotations,
                armTrajectorySetpointPositionRotations,
                armVelocityRPS,
                armAppliedVoltage.get(0),
                armAppliedVoltage.get(1),
                armOutputCurrent.get(0),
                armOutputCurrent.get(1),
                armTorqueCurrent.get(0),
                armTorqueCurrent.get(1),
                armTempCelsius.get(0),
                armTempCelsius.get(1));

        inputs.armAnglePositionRads = Units.rotationsToRadians(armPositionRotations.getValue());
        inputs.armTrajectorySetpointRads =
                Units.rotationsToRadians(armTrajectorySetpointPositionRotations.getValue());
        inputs.armVelocityRadsPerSec = Units.rotationsToRadians(armVelocityRPS.getValue());
        inputs.armAppliedVolts =
                armAppliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.armCurrentAmps =
                armOutputCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.armTorqueCurrentAmps =
                armTorqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.armTempCelcius =
                armTempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
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
