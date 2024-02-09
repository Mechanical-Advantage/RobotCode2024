package org.littletonrobotics.frc2024.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.Getter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.moduleConstants;
import static org.littletonrobotics.frc2024.subsystems.drive.DriveConstants.driveConfig;

public class Module {
    private static final LoggedTunableNumber drivekP =
            new LoggedTunableNumber("Drive/Module/DrivekP", moduleConstants.drivekP());
    private static final LoggedTunableNumber drivekD =
            new LoggedTunableNumber("Drive/Module/DrivekD", moduleConstants.drivekD());
    private static final LoggedTunableNumber drivekS =
            new LoggedTunableNumber("Drive/Module/DrivekS", moduleConstants.ffkS());
    private static final LoggedTunableNumber drivekV =
            new LoggedTunableNumber("Drive/Module/DrivekV", moduleConstants.ffkV());
    private static final LoggedTunableNumber turnkP =
            new LoggedTunableNumber("Drive/Module/TurnkP", moduleConstants.turnkP());
    private static final LoggedTunableNumber turnkD =
            new LoggedTunableNumber("Drive/Module/TurnkD", moduleConstants.turnkD());

    private final int index;
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SimpleMotorFeedforward driveFF =
            new SimpleMotorFeedforward(
                    moduleConstants.ffkS(), moduleConstants.ffkV(), 0.0);

    @Getter private SwerveModuleState setpointState = new SwerveModuleState();

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    /** Called while blocking odometry thread */
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);

        // Update FF and controllers
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    driveFF = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(), 0);
                    io.setDriveFF(drivekS.get(), drivekV.get(), 0);
                },
                drivekS,
                drivekV);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP, drivekD);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setTurnPID(turnkP.get(), 0, turnkD.get()), turnkP, turnkD);
    }

    public void runSetpoint(SwerveModuleState setpoint) {
        setpointState = setpoint;
        io.setDriveVelocitySetpoint(
                setpoint.speedMetersPerSecond / driveConfig.wheelRadius(),
                driveFF.calculate(setpoint.speedMetersPerSecond / driveConfig.wheelRadius()));
        io.setTurnPositionSetpoint(setpoint.angle.getRadians());
    }

    public void runCharacterization(double volts) {
        io.setTurnPositionSetpoint(0.0);
        io.setDriveVoltage(volts);
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    public void stop() {
        io.stop();
    }

    public SwerveModulePosition[] getModulePositions() {
        int minOdometryPositions =
                Math.min(inputs.odometryDrivePositionsMeters.length, inputs.odometryTurnPositions.length);
        SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
        for (int i = 0; i < minOdometryPositions; i++) {
            positions[i] =
                    new SwerveModulePosition(
                            inputs.odometryDrivePositionsMeters[i], inputs.odometryTurnPositions[i]);
        }
        return positions;
    }

    public Rotation2d getAngle() {
        return inputs.turnAbsolutePosition;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * driveConfig.wheelRadius();
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * driveConfig.wheelRadius();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
