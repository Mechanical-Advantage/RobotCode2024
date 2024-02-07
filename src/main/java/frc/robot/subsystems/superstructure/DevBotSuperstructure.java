package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DevBotSuperstructure extends SubsystemBase {
  public enum State {
    SHOOTING,
    INTAKING,
    IDLE
  }

  @Getter private State currentState = State.IDLE;
  private final Arm arm;
  private final Shooter shooter;

  public DevBotSuperstructure(Arm arm, Shooter shooter) {
    this.arm = arm;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLE -> {
        arm.setSetpoint(Units.degreesToRadians(5.0));
        shooter.stop();
      }
      case INTAKING -> {
        arm.setSetpoint(Units.degreesToRadians(50.0));
        if (arm.atSetpoint()) {
          shooter.setIntaking();
        }
      }
      case SHOOTING -> {
        if (arm.atSetpoint()) {
          shooter.setShooting();
        }
      }
    }

    Logger.recordOutput("DevBotSuperstructure/currentState", currentState.toString());
  }

  public void setDesiredState(State desiredState) {
    currentState = desiredState;
  }
}
