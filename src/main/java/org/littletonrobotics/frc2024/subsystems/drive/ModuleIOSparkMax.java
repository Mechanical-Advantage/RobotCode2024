// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.littletonrobotics.frc2024.subsystems.drive;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;
import java.util.function.Supplier;

public class ModuleIOSparkMax implements ModuleIO {
    // Hardware
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final AnalogInput turnAbsoluteEncoder;

    // Controllers
    private final PIDController driveController;
    private final PIDController turnController;

    // Queues
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final Rotation2d absoluteEncoderOffset;
    private final Supplier<Rotation2d> absoluteEncoderValue;
    private static final int shouldResetCount = 100;
    private int resetCounter = shouldResetCount;

    public ModuleIOSparkMax(ModuleConfig config) {
        // Init motor & encoder objects
        driveMotor = new CANSparkMax(config.driveID(), CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(config.turnID(), CANSparkMax.MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(config.absoluteEncoderChannel());
        absoluteEncoderOffset = config.absoluteEncoderOffset();
        driveEncoder = driveMotor.getEncoder();
        turnRelativeEncoder = turnMotor.getEncoder();

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        driveMotor.setCANTimeout(250);
        turnMotor.setCANTimeout(250);

        for (int i = 0; i < 30; i++) {
            turnMotor.setInverted(config.turnMotorInverted());
            driveMotor.setSmartCurrentLimit(40);
            turnMotor.setSmartCurrentLimit(30);
            driveMotor.enableVoltageCompensation(12.0);
            turnMotor.enableVoltageCompensation(12.0);

            driveEncoder.setPosition(0.0);
            driveEncoder.setMeasurementPeriod(10);
            driveEncoder.setAverageDepth(2);

            turnRelativeEncoder.setPosition(0.0);
            turnRelativeEncoder.setMeasurementPeriod(10);
            turnRelativeEncoder.setAverageDepth(2);

            driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 / odometryFrequency));
            turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 / odometryFrequency));

            if (driveMotor.burnFlash().equals(REVLibError.kOk)
                    && turnMotor.burnFlash().equals(REVLibError.kOk)) break;
        }
        driveMotor.setCANTimeout(0);
        turnMotor.setCANTimeout(0);

        absoluteEncoderValue =
                () ->
                        Rotation2d.fromRotations(
                                        turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V())
                                .minus(absoluteEncoderOffset);

        drivePositionQueue =
                SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
        turnPositionQueue =
                SparkMaxOdometryThread.getInstance()
                        .registerSignal(() -> absoluteEncoderValue.get().getRadians());

        // Init Controllers
        driveController = new PIDController(moduleConstants.drivekP(), 0.0, moduleConstants.drivekD());
        turnController = new PIDController(moduleConstants.turnkP(), 0.0, moduleConstants.turnkD());
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad =
                Units.rotationsToRadians(driveEncoder.getPosition() / moduleConstants.driveReduction());
        inputs.driveVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        driveEncoder.getVelocity() / moduleConstants.driveReduction());
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveSupplyCurrentAmps = driveMotor.getOutputCurrent();

        inputs.turnAbsolutePosition = absoluteEncoderValue.get();
        inputs.turnPosition =
                Rotation2d.fromRadians(
                        Units.rotationsToRadians(
                                turnRelativeEncoder.getPosition() / moduleConstants.turnReduction()));
        inputs.turnVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        turnRelativeEncoder.getVelocity() / moduleConstants.turnReduction());
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnSupplyCurrentAmps = turnMotor.getOutputCurrent();

        inputs.odometryDrivePositionsMeters =
                drivePositionQueue.stream()
                        .mapToDouble(
                                motorPositionRevs ->
                                        Units.rotationsToRadians(motorPositionRevs / moduleConstants.driveReduction())
                                                * driveConfig.wheelRadius())
                        .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    @Override
    public void setDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
        double feedback =
                driveController.calculate(
                        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
                                / moduleConstants.driveReduction(),
                        velocityRadsPerSec);
        setDriveVoltage(feedback + ffVolts);
    }

    @Override
    public void setTurnPositionSetpoint(double angleRads) {
        setTurnVoltage(turnController.calculate(absoluteEncoderValue.get().getRadians(), angleRads));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        driveController.setPID(kP, kI, kD);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        turnController.setPID(kP, kI, kD);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
