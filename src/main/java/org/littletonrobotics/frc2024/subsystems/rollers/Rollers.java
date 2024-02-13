package org.littletonrobotics.frc2024.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.Indexer;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private final Feeder feeder;
  private final Indexer indexer;
  private final Intake intake;

  private final RollersSensorsIO sensorsIO;
  private final RollersSensorsIOInputsAutoLogged sensorInputs =
      new RollersSensorsIOInputsAutoLogged();

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    EJECT_TO_FLOOR,
    FEED_SHOOTER
  }

  @Getter @Setter private Goal goal = Goal.IDLE;

  public Rollers(Feeder feeder, Indexer indexer, Intake intake, RollersSensorsIO sensorsIO) {
    this.feeder = feeder;
    this.indexer = indexer;
    this.intake = intake;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(idleCommand());
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    feeder.periodic();
    indexer.periodic();
    intake.periodic();
  }

  public Command idleCommand() {
    return runOnce(
            () -> {
              feeder.setGoal(Feeder.Goal.IDLE);
              indexer.setGoal(Indexer.Goal.IDLE);
              intake.setGoal(Intake.Goal.IDLE);
            })
        .andThen(Commands.idle())
        .withName("Rollers Idle");
  }

  public Command floorIntakeCommand() {
    return runOnce(
            () -> {
              feeder.setGoal(Feeder.Goal.FLOOR_INTAKING);
              indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
              intake.setGoal(Intake.Goal.FLOOR_INTAKING);
              if (sensorInputs.shooterStaged) {
                indexer.setGoal(Indexer.Goal.IDLE);
              }
            })
        .andThen(Commands.idle())
        .withName("Rollers Floor Intake");
  }

  public Command stationIntakeCommand() {
    return runOnce(
            () -> {
              feeder.setGoal(Feeder.Goal.IDLE);
              indexer.setGoal(Indexer.Goal.STATION_INTAKING);
              intake.setGoal(Intake.Goal.IDLE);
              if (sensorInputs.shooterStaged) { // TODO: ADD THIS BANNER
                indexer.setGoal(Indexer.Goal.IDLE);
              }
            })
        .andThen(Commands.idle())
        .withName("Rollers Station Intake");
  }

  public Command ejectFloorCommand() {
    return runOnce(
            () -> {
              feeder.setGoal(Feeder.Goal.EJECTING);
              indexer.setGoal(Indexer.Goal.EJECTING);
              intake.setGoal(Intake.Goal.EJECTING);
            })
        .andThen(Commands.idle())
        .withName("Rollers Eject Floor");
  }

  public Command feedShooterCommand() {
    return runOnce(
            () -> {
              feeder.setGoal(Feeder.Goal.SHOOTING);
              indexer.setGoal(Indexer.Goal.SHOOTING);
              intake.setGoal(Intake.Goal.IDLE);
            })
        .andThen(Commands.idle())
        .withName("Rollers Feed Shooter");
  }
}