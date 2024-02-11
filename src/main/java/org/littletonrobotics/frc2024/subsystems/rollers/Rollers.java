package org.littletonrobotics.frc2024.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.Indexer;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor()
public class Rollers extends SubsystemBase {
  private final Feeder feeder;
  private final Indexer indexer;
  private final Intake intake;
  private final RollersSensorsIO sensorsIO;

  private final RollersSensorsIOInputsAutoLogged inputs = new RollersSensorsIOInputsAutoLogged();

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    EJECT_TO_FLOOR,
    FEED_SHOOTER
  }

  @Getter @Setter private Goal goal = Goal.IDLE;

  @Override
  public void periodic() {
    sensorsIO.updateInputs(inputs);
    Logger.processInputs("RollersSensors", inputs);

    switch (goal) {
      case IDLE -> {
        feeder.setGoal(Feeder.Goal.IDLE);
        indexer.setGoal(Indexer.Goal.IDLE);
        intake.setGoal(Intake.Goal.IDLE);
      }
      case FLOOR_INTAKE -> {
        feeder.setGoal(Feeder.Goal.FLOOR_INTAKING);
        indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
        if (inputs.shooterStaged) {
          goal = Goal.IDLE;
        }
      }
      case STATION_INTAKE -> {
        indexer.setGoal(Indexer.Goal.STATION_INTAKING);
      }
      case FEED_SHOOTER -> indexer.setGoal(Indexer.Goal.SHOOTING);
      case EJECT_TO_FLOOR -> {
        feeder.setGoal(Feeder.Goal.EJECTING);
        indexer.setGoal(Indexer.Goal.EJECTING);
        intake.setGoal(Intake.Goal.EJECTING);
      }
    }

    Logger.recordOutput("Rollers/Goal", goal);

    feeder.periodic();
    indexer.periodic();
    intake.periodic();
  }
}
