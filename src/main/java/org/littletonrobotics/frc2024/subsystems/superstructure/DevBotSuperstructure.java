package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.feeder.Feeder;
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
    private final Feeder feeder;

    private final Timer followThroughTimer = new Timer();

    public DevBotSuperstructure(Arm arm, Flywheels flywheels, Feeder feeder) {
        this.arm = arm;
        this.flywheels = flywheels;
        this.feeder = feeder;
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
                    goalState = SystemState.IDLE;
                }
            }
        }

        switch (currentState) {
            case IDLE -> {
                arm.setSetpoint(Rotation2d.fromDegrees(armIdleSetpointDegrees.get()));
                flywheels.runVolts(4.0, 4.0);
                feeder.stop();
            }
            case INTAKE -> {
                arm.setSetpoint(Rotation2d.fromDegrees(armIntakeSetpointDegrees.get()));
                flywheels.runVolts(-2.0, -2.0);
                feeder.runVolts(-0.5);
            }
            case PREPARE_SHOOT -> {
                var aimingParams = RobotState.getInstance().getAimingParameters();
                arm.setSetpoint(
                        new Rotation2d(
                                aimingParams.effectiveDistance(),
                                FieldConstants.Speaker.centerSpeakerOpening.getZ()
                                        + Units.inchesToMeters(yCompensation.get())));
                flywheels.setSetpointRpm(shootingLeftRPM.get(), shootingRightRPM.get());
                feeder.runVolts(0.0);
            }
            case SHOOT -> {
                feeder.runVolts(2.0);
            }
        }

        Logger.recordOutput("DevBotSuperstructure/GoalState", goalState);
        Logger.recordOutput("DevBotSuperstructure/currentState", currentState);
    }

    @AutoLogOutput(key = "DevBotSuperstructure/ReadyToShoot")
    public boolean atShootingSetpoint() {
        return flywheels.atSetpoint();
    }
}
