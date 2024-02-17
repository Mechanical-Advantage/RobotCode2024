// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.Indexer;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.frc2024.util.NoteVisualizer;
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
    QUICK_INTAKE_TO_FEED,
    FEED_TO_SHOOTER
  }

  @Getter private Goal goal = Goal.IDLE;

  public Rollers(Feeder feeder, Indexer indexer, Intake intake, RollersSensorsIO sensorsIO) {
    this.feeder = feeder;
    this.indexer = indexer;
    this.intake = intake;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(runOnce(this::goIdle).withName("Rollers Idling"));
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    if (DriverStation.isDisabled()) {
      goIdle();
    }

    switch (goal) {
      case IDLE -> {
        feeder.setGoal(Feeder.Goal.IDLING);
        indexer.setGoal(Indexer.Goal.IDLING);
        intake.setGoal(Intake.Goal.IDLING);
      }
      case FLOOR_INTAKE -> {
        feeder.setGoal(Feeder.Goal.FLOOR_INTAKING);
        indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
        if (sensorInputs.shooterStaged) {
          indexer.setGoal(Indexer.Goal.IDLING);
        }
      }
      case STATION_INTAKE -> {
        feeder.setGoal(Feeder.Goal.IDLING);
        indexer.setGoal(Indexer.Goal.STATION_INTAKING);
        intake.setGoal(Intake.Goal.IDLING);
        if (sensorInputs.shooterStaged) { // TODO: add this banner sensor
          indexer.setGoal(Indexer.Goal.IDLING);
        }
      }
      case EJECT_TO_FLOOR -> {
        feeder.setGoal(Feeder.Goal.EJECTING);
        indexer.setGoal(Indexer.Goal.EJECTING);
        intake.setGoal(Intake.Goal.EJECTING);
      }
      case QUICK_INTAKE_TO_FEED -> {
        feeder.setGoal(Feeder.Goal.SHOOTING);
        indexer.setGoal(Indexer.Goal.SHOOTING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
      }
      case FEED_TO_SHOOTER -> {
        feeder.setGoal(Feeder.Goal.SHOOTING);
        indexer.setGoal(Indexer.Goal.SHOOTING);
        intake.setGoal(Intake.Goal.IDLING);
      }
    }

    feeder.periodic();
    indexer.periodic();
    intake.periodic();
  }

  private void goIdle() {
    goal = Goal.IDLE;
  }

  public boolean hasGamepiece() {
    return sensorInputs.shooterStaged;
  }

  public Command floorIntake() {
    return startEnd(() -> goal = Goal.FLOOR_INTAKE, this::goIdle).withName("Rollers Floor Intake");
  }

  public Command stationIntake() {
    return startEnd(() -> goal = Goal.STATION_INTAKE, this::goIdle)
        .withName("Rollers Station Intake");
  }

  public Command ejectFloor() {
    return startEnd(() -> goal = Goal.EJECT_TO_FLOOR, this::goIdle).withName("Rollers Eject Floor");
  }

  public Command quickFeed() {
    return startEnd(() -> goal = Goal.QUICK_INTAKE_TO_FEED, this::goIdle)
        .withName("Rollers Quick Feed");
  }

  public Command feedShooter() {
    return startEnd(() -> goal = Goal.FEED_TO_SHOOTER, this::goIdle)
        .alongWith(NoteVisualizer.shoot())
        .withName("Rollers Feed Shooter");
  }
}
