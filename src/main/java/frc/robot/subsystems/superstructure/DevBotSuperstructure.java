package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
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
  private final Shooter shooter;

  private final Timer followThroughTimer = new Timer();
  private double followThroughArmAngle = 0.0;

  public DevBotSuperstructure(Arm arm, Shooter shooter) {
    this.arm = arm;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLE -> {
        arm.setSetpoint(Units.degreesToRadians(armIdleSetpointDegrees.get()));
        shooter.requestRPM(idleLeftRPM.get(), idleRightRPM.get());
      }
      case INTAKE -> {
        arm.setSetpoint(Units.degreesToRadians(armIntakeSetpointDegrees.get()));
        if (arm.atSetpoint()) {
          shooter.requestIntake();
        }
      }
      case PREPARE_SHOOT -> {
        var aimingParameters = RobotState.getInstance().getAimingParameters();
        double y = (FieldConstants.Speaker.centerSpeakerOpening.getY()) / 2.0;
        y += Units.inchesToMeters(yCompensation.get());
        double x = aimingParameters.effectiveDistance();
        arm.setSetpoint(Math.atan(y / x));
        shooter.requestRPM(shootingLeftRPM.get(), shootingRightRPM.get());
      }
      case SHOOT -> {
        if (!atShootingSetpoint()) {
          currentState = State.PREPARE_SHOOT;
        } else {
          shooter.requestFeed();
          followThroughArmAngle = arm.getAngle().getRadians();
          followThroughTimer.restart();
        }
      }
    }

    Logger.recordOutput("DevBotSuperstructure/currentState", currentState.toString());
  }

  @AutoLogOutput(key = "DevBotSuperstructure/ReadyToShoot")
  public boolean atShootingSetpoint() {
    return (currentState == State.PREPARE_SHOOT || currentState == State.SHOOT)
        && arm.atSetpoint()
        && shooter.atSetpoint();
  }

  public void setDesiredState(State desiredState) {
    if (desiredState == State.PREPARE_SHOOT && currentState == State.SHOOT) {
      return;
    }
    currentState = desiredState;
  }
}
