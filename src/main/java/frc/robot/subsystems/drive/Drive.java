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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.planners.TrajectoryMotionPlanner;
import frc.robot.util.GeomUtil;
import frc.robot.util.swerve.ModuleLimits;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;
import frc.robot.util.trajectory.Trajectory;

import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;

import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class Drive extends SubsystemBase {
    @AutoLog
    public static class OdometryTimeestampInputs {
        public double[] timestamps = new double[]{};
    }

    public static final Lock odometryLock = new ReentrantLock();
    // TODO: DO THIS BETTER!
    public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(100);

    private final OdometryTimeestampInputsAutoLogged odometryTimestampInputs =
            new OdometryTimeestampInputsAutoLogged();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];

  private ControlMode currentControlMode = ControlMode.DRIVER_INPUT;
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private final AutoMotionPlanner autoMotionPlanner;
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private final TrajectoryMotionPlanner trajectoryMotionPlanner;

    private ModuleLimits currentModuleLimits = DriveConstants.moduleLimits;
    private SwerveSetpoint currentSetpoint =
            new SwerveSetpoint(
                    new ChassisSpeeds(),
                    new SwerveModuleState[]{
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState()
                    });
    private SwerveSetpointGenerator setpointGenerator;

    private boolean characterizing = false;
    private double characterizationVolts = 0.0;

    public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        System.out.println("[Init] Creating Drive");
        this.gyroIO = gyroIO;
        modules[0] = new Module(fl, 0);
        modules[1] = new Module(fr, 1);
        modules[2] = new Module(bl, 2);
        modules[3] = new Module(br, 3);

    setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(DriveConstants.kinematics)
            .moduleLocations(DriveConstants.moduleTranslations)
            .build();
    autoMotionPlanner = new AutoMotionPlanner();
  }
        setpointGenerator =
                SwerveSetpointGenerator.builder()
                        .kinematics(DriveConstants.kinematics)
                        .moduleLocations(DriveConstants.moduleTranslations)
                        .build();
        trajectoryMotionPlanner = new TrajectoryMotionPlanner();
    }

    public void periodic() {
        // Update & process inputs
        odometryLock.lock();
        // Read timestamps from odometry thread and fake sim timestamps
        odometryTimestampInputs.timestamps = timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
        if (odometryTimestampInputs.timestamps.length == 0) {
            odometryTimestampInputs.timestamps = new double[]{Timer.getFPGATimestamp()};
        }
        timestampQueue.clear();
        Logger.processInputs("Drive/OdometryTimestamps", odometryTimestampInputs);
        // Read inputs from gyro
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        // Read inputs from modules
        Arrays.stream(modules).forEach(Module::updateInputs);
        odometryLock.unlock();

        // Calculate the min odometry position updates across all modules
        int minOdometryUpdates =
                IntStream.of(
                                odometryTimestampInputs.timestamps.length,
                                Arrays.stream(modules)
                                        .mapToInt(module -> module.getModulePositions().length)
                                        .min()
                                        .orElse(0))
                        .min()
                        .orElse(0);
        if (gyroInputs.connected) {
            minOdometryUpdates = Math.min(gyroInputs.odometryYawPositions.length, minOdometryUpdates);
        }
        // Pass odometry data to robot state
        for (int i = 0; i < minOdometryUpdates; i++) {
            int odometryIndex = i;
            Rotation2d yaw = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;
            // Get all four swerve module positions at that odometry update
            // and store in SwerveDriveWheelPositions object
            SwerveDriveWheelPositions wheelPositions =
                    new SwerveDriveWheelPositions(
                            Arrays.stream(modules)
                                    .map(module -> module.getModulePositions()[odometryIndex])
                                    .toArray(SwerveModulePosition[]::new));
            // Add observation to robot state
            RobotState.getInstance()
                    .addOdometryObservation(
                            new RobotState.OdometryObservation(
                                    wheelPositions,
                                    yaw,
                                    odometryTimestampInputs.timestamps[i]));
        }

        // update current velocities use gyro when possible
        ChassisSpeeds robotRelativeVelocity =
                DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
        robotRelativeVelocity.omegaRadiansPerSecond =
                gyroInputs.connected
                        ? gyroInputs.yawVelocityRadPerSec
                        : robotRelativeVelocity.omegaRadiansPerSecond;
        RobotState.getInstance().addVelocityData(robotRelativeVelocity.toTwist2d());

        // Run characterization
        if (characterizing) {
            for (Module module : modules) {
                module.runCharacterization(characterizationVolts);
            }

            Logger.recordOutput("Drive/SwerveStates/Desired(b4 Poofs)", new double[]{});
            Logger.recordOutput("Drive/SwerveStates/Setpoints", new double[]{});
            Logger.recordOutput("Drive/DesiredSpeeds", new double[]{});
            Logger.recordOutput("Drive/SetpointSpeeds", new double[]{});
            return;
        }

        // Run robot at speeds
        // Generate feasible next setpoint
        currentSetpoint =
                setpointGenerator.generateSetpoint(
                        currentModuleLimits, currentSetpoint, desiredSpeeds, 0.02);

        // run modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            // Optimize setpoints
            optimizedSetpointStates[i] =
                    SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());
            modules[i].runSetpoint(optimizedSetpointStates[i]);
        }

        // Log chassis speeds and swerve states
        Logger.recordOutput(
                "Drive/SwerveStates/Desired(b4 Poofs)",
                DriveConstants.kinematics.toSwerveModuleStates(desiredSpeeds));
        Logger.recordOutput("Drive/SwerveStates/Setpoints", optimizedSetpointStates);
        Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
        Logger.recordOutput("Drive/SetpointSpeeds", currentSetpoint.chassisSpeeds());
    }

    /**
     * Set drive velocity (robot relative)
     */
    public void setVelocity(ChassisSpeeds velocity) {
        desiredSpeeds = ChassisSpeeds.discretize(velocity, 0.02);
    }

    /**
     * Runs forwards at the commanded voltage.
     */
    public void runCharacterizationVolts(double volts) {
        characterizing = true;
        characterizationVolts = volts;
    }

    public void endCharacterization() {
        characterizing = false;
    }

    /**
     * Returns the average drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    public void setBrakeMode(boolean enabled) {
        Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }

    public Command followTrajectory(Trajectory trajectory) {
        return Commands.sequence(
                Commands.runOnce(() -> trajectoryMotionPlanner.setTrajectory(trajectory)),
                Commands.run(() -> setVelocity(trajectoryMotionPlanner.update()))
                        .until(trajectoryMotionPlanner::isFinished));
    }

    @AutoLogOutput(key = "Drive/GyroYaw")
    public Rotation2d getGyroYaw() {
        return gyroInputs.connected
                ? gyroInputs.yawPosition
                : RobotState.getInstance().getEstimatedPose().getRotation();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "Drive/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    }

    public static Rotation2d[] getStraightOrientations() {
        return IntStream.range(0, 4).mapToObj(Rotation2d::new).toArray(Rotation2d[]::new);
    }

    public static Rotation2d[] getXOrientations() {
        return Arrays.stream(DriveConstants.moduleTranslations)
                .map(Translation2d::getAngle)
                .toArray(Rotation2d[]::new);
    }
}
