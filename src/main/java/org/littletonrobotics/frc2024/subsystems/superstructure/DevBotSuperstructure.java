package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.flywheels.Flywheels;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DevBotSuperstructure extends SubsystemBase {
    private static LoggedTunableNumber armIdleSetpointDegrees =
            new LoggedTunableNumber("DevBotSuperstructure/ArmIdleSetpointDegrees", 10.0);
    private static LoggedTunableNumber armIntakeSetpointDegrees =
            new LoggedTunableNumber("DevBotSuperstructure/ArmIntakeSetpointDegrees", 50.0);
    private static LoggedTunableNumber shootingLeftRPM =
            new LoggedTunableNumber("DevBotSuperstructure/ShootingLeftRPM", 6000.0);
    private static LoggedTunableNumber shootingRightRPM =
            new LoggedTunableNumber("DevBotSuperstructure/ShootingRightRPM", 4000.0);
    private static LoggedTunableNumber idleLeftRPM =
            new LoggedTunableNumber("DevBotSuperstructure/IdleLeftRPM", 2000.0);
    private static LoggedTunableNumber idleRightRPM =
            new LoggedTunableNumber("DevBotSuperstructure/IdleRightRPM", 2000.0);
    private static LoggedTunableNumber yCompensation =
            new LoggedTunableNumber("DevBotSuperstructure/CompensationInches", 6.0);
    private static LoggedTunableNumber followThroughTime =
            new LoggedTunableNumber("DevBotSuperstructure/FollowthroughTimeSecs", 0.5);

    public enum SystemState {
        PREPARE_SHOOT,
        SHOOT,
        INTAKE,
        IDLE
    }

    @Getter private SystemState currentState = SystemState.IDLE;
    @Getter @Setter private SystemState goalState = SystemState.IDLE;
    private final Arm arm;
    private final Flywheels flywheels;

    private final Timer followThroughTimer = new Timer();
    private double followThroughArmAngle = 0.0;

    public DevBotSuperstructure(Arm arm, Flywheels flywheels) {
        this.arm = arm;
        this.flywheels = flywheels;
    }

    @Override
    public void periodic() {
        switch (goalState) {
            case IDLE -> {
                if (currentState == SystemState.SHOOT) {
                    if (followThroughTimer.hasElapsed(followThroughTime.get())) {
                        currentState = SystemState.IDLE;
                        followThroughTimer.stop();
                        followThroughTimer.reset();
                    } else {
                        currentState = SystemState.SHOOT;
                    }
                } else {
                    currentState = SystemState.IDLE;
                }
            }
            case INTAKE -> currentState = SystemState.INTAKE;
            case PREPARE_SHOOT -> currentState = SystemState.PREPARE_SHOOT;
            case SHOOT -> {
                if (currentState != SystemState.PREPARE_SHOOT) {
                    currentState = SystemState.PREPARE_SHOOT;
                } else if (atShootingSetpoint()) {
                    currentState = SystemState.SHOOT;
                    followThroughTimer.restart();
                }
            }
        }

        switch (currentState) {
            case IDLE -> {
                arm.setSetpoint(Rotation2d.fromDegrees(armIdleSetpointDegrees.get()));
                flywheels.runVolts(0.0, 0.0);
            }
            case INTAKE -> {}
            case PREPARE_SHOOT -> {}
            case SHOOT -> {}
        }

        Logger.recordOutput("DevBotSuperstructure/GoalState", goalState);
        Logger.recordOutput("DevBotSuperstructure/currentState", currentState);
    }

    @AutoLogOutput(key = "DevBotSuperstructure/ReadyToShoot")
    public boolean atShootingSetpoint() {
        return arm.atSetpoint() && flywheels.atSetpoint();
    }
}
