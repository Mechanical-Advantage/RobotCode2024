package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
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
            new LoggedTunableNumber("DevBotSuperstructure/FollowthroughTimeSecs", 1.0);

    public enum State {
        PREPARE_SHOOT,
        SHOOT,
        INTAKE,
        IDLE
    }

    @Getter private State currentState = State.IDLE;
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
        switch (currentState) {
            case IDLE -> {}
            case INTAKE -> {}
            case PREPARE_SHOOT -> {}
            case SHOOT -> {}
        }

        Logger.recordOutput("DevBotSuperstructure/currentState", currentState.toString());
    }

    @AutoLogOutput(key = "DevBotSuperstructure/ReadyToShoot")
    public boolean atShootingSetpoint() {
        return (currentState == State.PREPARE_SHOOT || currentState == State.SHOOT)
                && arm.atSetpoint()
                && flywheels.atSetpoint();
    }

    public void setDesiredState(State desiredState) {
        if (desiredState == State.PREPARE_SHOOT && currentState == State.SHOOT) {
            return;
        }
        currentState = desiredState;
    }
}
