package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;
import java.util.function.Supplier;

public class ModuleIOKrakenFOC implements ModuleIO {
    // Hardware
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final AnalogInput turnAbsoluteEncoder;
    private final Rotation2d absoluteEncoderOffset;

    // Status Signals
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTorqueCurrent;

    private final StatusSignal<Double> turnPosition;
    private final Supplier<Rotation2d> turnAbsolutePosition;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnSupplyCurrent;
    private final StatusSignal<Double> turnTorqueCurrent;

    // Queues
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private static final int shouldResetCounts = 100;
    private int resetCounter = shouldResetCounts;

    private Slot0Configs driveFeedbackConfig = new Slot0Configs();
    private Slot0Configs turnFeedbackConfig = new Slot0Configs();

    public ModuleIOKrakenFOC(ModuleConfig config) {
        // Init controllers and encoders from config constants
        driveTalon = new TalonFX(config.driveID(), "canivore");
        turnTalon = new TalonFX(config.turnID(), "canivore");
        turnAbsoluteEncoder = new AnalogInput(config.absoluteEncoderChannel());
        absoluteEncoderOffset = config.absoluteEncoderOffset();

        // Config Motors
        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Voltage.PeakForwardVoltage = 12.0;
        driveConfig.Voltage.PeakReverseVoltage = -12.0;

        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Voltage.PeakForwardVoltage = 12.0;
        turnConfig.Voltage.PeakReverseVoltage = -12.0;
        turnConfig.MotorOutput.Inverted =
                config.turnMotorInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        // If in motoControl mode, set reference points in rotations convert from radians
        // Affects getPosition() and getVelocity()
        driveConfig.Feedback.SensorToMechanismRatio = moduleConstants.driveReduction();
        turnConfig.Feedback.SensorToMechanismRatio = moduleConstants.turnReduction();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // Config Motion Magic
        if (KrakenDriveConstants.useMotionMagic) {
            turnConfig.MotionMagic.MotionMagicCruiseVelocity =
                    KrakenDriveConstants.motionMagicCruiseVelocity;
            turnConfig.MotionMagic.MotionMagicAcceleration = KrakenDriveConstants.motionMagicAcceleration;
        }

        // Apply configs
        for (int i = 0; i < 4; i++) {
            boolean error = driveTalon.getConfigurator().apply(driveConfig, 0.1) == StatusCode.OK;
            setDriveBrakeMode(true);
            error = error && (turnTalon.getConfigurator().apply(turnConfig, 0.1) == StatusCode.OK);
            setTurnBrakeMode(true);
            if (!error) break;
        }

        // Get signals and set update rate
        // 100hz signals
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveSupplyCurrent = driveTalon.getSupplyCurrent();
        driveTorqueCurrent = driveTalon.getTorqueCurrent();
        turnAbsolutePosition =
                () ->
                        Rotation2d.fromRadians(
                                        turnAbsoluteEncoder.getVoltage()
                                                / RobotController.getVoltage5V()
                                                * 2.0
                                                * Math.PI)
                                .minus(absoluteEncoderOffset);
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnSupplyCurrent = turnTalon.getSupplyCurrent();
        turnTorqueCurrent = turnTalon.getTorqueCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrent,
                turnTorqueCurrent);

        // 250hz signals
        drivePosition = driveTalon.getPosition();
        turnPosition = turnTalon.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition, turnPosition);
        drivePositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
        turnPositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());

        // TODO: what does this do?
        //        driveTalon.optimizeBusUtilization();
        //        turnTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Reset position of encoder to absolute position every shouldResetCount cycles
        // Make sure turnMotor is not moving too fast
        if (++resetCounter >= shouldResetCounts
                && Units.rotationsToRadians(turnVelocity.getValueAsDouble()) <= 0.1) {
            turnTalon.setPosition(turnAbsolutePosition.get().getRotations());
            resetCounter = 0;
        }

        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrent,
                turnTorqueCurrent);

        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
        inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

        inputs.turnAbsolutePosition = turnAbsolutePosition.get();
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
        inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();

        inputs.odometryDrivePositionsMeters =
                drivePositionQueue.stream()
                        .mapToDouble(signalValue -> Units.rotationsToRadians(signalValue) * wheelRadius)
                        .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts).withEnableFOC(true));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnTalon.setControl(new VoltageOut(volts).withEnableFOC(true));
    }

    @Override
    public void setDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
        double velocityRotationsPerSec = Units.radiansToRotations(velocityRadsPerSec);
        if (KrakenDriveConstants.useTorqueCurrentFOC) {
            driveTalon.setControl(new VelocityTorqueCurrentFOC(velocityRotationsPerSec));
        } else {
            driveTalon.setControl(
                    new VelocityVoltage(velocityRotationsPerSec)
                            .withFeedForward(ffVolts)
                            .withEnableFOC(true));
        }
    }

    @Override
    public void setTurnPositionSetpoint(double angleRads) {
        double angleRotations = Units.radiansToRotations(angleRads);
        if (KrakenDriveConstants.useTorqueCurrentFOC) {
            if (KrakenDriveConstants.useMotionMagic) {
                turnTalon.setControl(new MotionMagicTorqueCurrentFOC(angleRotations));
            } else {
                turnTalon.setControl(new PositionTorqueCurrentFOC(angleRotations));
            }
        } else {
            if (KrakenDriveConstants.useMotionMagic) {
                turnTalon.setControl(new MotionMagicVoltage(angleRotations).withEnableFOC(true));
            } else {
                turnTalon.setControl(new PositionVoltage(angleRotations).withEnableFOC(true));
            }
        }
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        driveFeedbackConfig.kP = kP;
        driveFeedbackConfig.kI = kI;
        driveFeedbackConfig.kD = kD;
        driveTalon.getConfigurator().apply(driveFeedbackConfig, 0.01);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        turnFeedbackConfig.kP = kP;
        turnFeedbackConfig.kI = kI;
        turnFeedbackConfig.kD = kD;
        turnTalon.getConfigurator().apply(turnFeedbackConfig, 0.01);
    }

    @Override
    public void setDriveFF(double kS, double kV, double kA) {
        driveFeedbackConfig.kS = kS;
        driveFeedbackConfig.kV = kV;
        driveFeedbackConfig.kA = kA;
        driveTalon.getConfigurator().apply(driveFeedbackConfig, 0.01);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void stop() {
        driveTalon.setControl(new NeutralOut());
        turnTalon.setControl(new NeutralOut());
    }
}
