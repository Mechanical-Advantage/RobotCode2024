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

public class Superstructure extends SubsystemBase {
    private static LoggedTunableNumber armIdleSetpointDegrees =
            new LoggedTunableNumber("Superstructure/ArmIdleSetpointDegrees", 20.0);
    private static LoggedTunableNumber armStationIntakeSetpointDegrees =
            new LoggedTunableNumber("Superstructure/ArmStationIntakeSetpointDegrees", 45.0);
    private static LoggedTunableNumber armIntakeSetpointDegrees =
            new LoggedTunableNumber("Superstructure/ArmIntakeDegrees", 20.0);
    private static LoggedTunableNumber yCompensation =
            new LoggedTunableNumber("Superstructure/CompensationInches", 6.0);
    private static LoggedTunableNumber followThroughTime =
            new LoggedTunableNumber("Superstructure/FollowthroughTimeSecs", 0.5);

    public enum SystemState {
        PREPARE_SHOOT,
        SHOOT,
        INTAKE,
        IDLE,
        STATION_INTAKE
    }

    @Getter private SystemState currentState = SystemState.IDLE;
    @Getter @Setter private SystemState goalState = SystemState.IDLE;
    private final Arm arm;
    private final Flywheels flywheels;
    private final Feeder feeder;

    private final Timer followThroughTimer = new Timer();

    public Superstructure(Arm arm, Flywheels flywheels, Feeder feeder) {
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
                flywheels.setGoal(Flywheels.Goal.IDLE);
                feeder.setGoal(Feeder.Goal.INTAKING);
            }
            case STATION_INTAKE -> {
                arm.setSetpoint(Rotation2d.fromDegrees(armStationIntakeSetpointDegrees.get()));
                flywheels.setGoal(Flywheels.Goal.INTAKING);
                feeder.setGoal(Feeder.Goal.REVERSE_INTAKING);
            }
            case PREPARE_SHOOT -> {
                var aimingParams = RobotState.getInstance().getAimingParameters();
                arm.setSetpoint(
                        new Rotation2d(
                                aimingParams.effectiveDistance(),
                                FieldConstants.Speaker.centerSpeakerOpening.getZ()
                                        + Units.inchesToMeters(yCompensation.get())));
                flywheels.setGoal(Flywheels.Goal.SHOOTING);
                feeder.setGoal(Feeder.Goal.IDLE);
            }
            case SHOOT -> {
                feeder.setGoal(Feeder.Goal.FEEDING);
            }
        }

        Logger.recordOutput("Superstructure/GoalState", goalState);
        Logger.recordOutput("Superstructure/CurrentState", currentState);
    }

    @AutoLogOutput(key = "Superstructure/ReadyToShoot")
    public boolean atShootingSetpoint() {
        return flywheels.atSetpoint();
    }
}
