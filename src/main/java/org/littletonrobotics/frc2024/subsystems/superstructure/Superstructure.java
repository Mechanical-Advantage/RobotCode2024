package org.littletonrobotics.frc2024.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class Superstructure extends SubsystemBase {
  private static LoggedTunableNumber armIdleSetpointDegrees =
      new LoggedTunableNumber("Superstructure/ArmIdleSetpointDegrees", 20.0);
  private static LoggedTunableNumber armStationIntakeSetpointDegrees =
      new LoggedTunableNumber("Superstructure/ArmStationIntakeSetpointDegrees", 45.0);
  private static LoggedTunableNumber armIntakeSetpointDegrees =
      new LoggedTunableNumber("Superstructure/ArmIntakeDegrees", 40.0);
  private static LoggedTunableNumber followThroughTime =
      new LoggedTunableNumber("Superstructure/FollowthroughTimeSecs", 0.5);

  public enum SystemState {
    PREPARE_SHOOT,
    SHOOT,
    PREPARE_INTAKE,
    INTAKE,
    STATION_INTAKE,
    REVERSE_INTAKE,
    IDLE
  }

  public enum GamepieceState {
    NO_GAMEPIECE,

    HOLDING_SHOOTER,

    HOLDING_BACKPACK
  }

  @Getter private SystemState currentState = SystemState.IDLE;
  @Getter @Setter private SystemState goalState = SystemState.IDLE;

  @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NO_GAMEPIECE;

  private final Arm arm;
  private final Flywheels flywheels;

  private final Timer followThroughTimer = new Timer();

  @Override
  public void periodic() {
    switch (goalState) {
      case IDLE -> currentState = SystemState.IDLE;
      case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
      case INTAKE -> currentState = SystemState.INTAKE;
      case PREPARE_SHOOT -> currentState = SystemState.PREPARE_SHOOT;
      case SHOOT -> currentState = SystemState.SHOOT;
    }

    switch (currentState) {
      case IDLE -> {
        arm.setSetpoint(Rotation2d.fromDegrees(armIdleSetpointDegrees.get()));
        flywheels.setGoal(Flywheels.Goal.IDLE);
      }
      case INTAKE -> {
        arm.setSetpoint(Rotation2d.fromDegrees(armIntakeSetpointDegrees.get()));
        flywheels.setGoal(Flywheels.Goal.IDLE);
      }
      case STATION_INTAKE -> {
        arm.setSetpoint(Rotation2d.fromDegrees(armStationIntakeSetpointDegrees.get()));
        flywheels.setGoal(Flywheels.Goal.INTAKING);
      }
      case REVERSE_INTAKE -> {
        arm.setSetpoint(Rotation2d.fromDegrees(armIntakeSetpointDegrees.get()));
      }
      case PREPARE_SHOOT -> {
        var aimingParams = RobotState.getInstance().getAimingParameters();
        arm.setSetpoint(aimingParams.armAngle());
        flywheels.setGoal(Flywheels.Goal.SHOOTING);
      }
      case SHOOT -> {
        var aimingParams = RobotState.getInstance().getAimingParameters();
        flywheels.setGoal(Flywheels.Goal.SHOOTING);
        arm.setSetpoint(aimingParams.armAngle());
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
