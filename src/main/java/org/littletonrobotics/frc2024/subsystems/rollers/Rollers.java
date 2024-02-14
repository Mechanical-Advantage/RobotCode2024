// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
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
    IDLING,
    FLOOR_INTAKING,
    STATION_INTAKING,
    EJECTING_TO_FLOOR,
    FEEDING_TO_SHOOTER
  }

  @Getter @Setter private Goal goal = Goal.IDLING;

  public Rollers(Feeder feeder, Indexer indexer, Intake intake, RollersSensorsIO sensorsIO) {
    this.feeder = feeder;
    this.indexer = indexer;
    this.intake = intake;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    switch (goal) {
      case IDLING -> {
        feeder.setGoal(Feeder.Goal.IDLING);
        indexer.setGoal(Indexer.Goal.IDLING);
        intake.setGoal(Intake.Goal.IDLING);
      }
      case FLOOR_INTAKING -> {
        feeder.setGoal(Feeder.Goal.FLOOR_INTAKING);
        indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
        if (sensorInputs.shooterStaged) {
          indexer.setGoal(Indexer.Goal.IDLING);
        }
      }
      case STATION_INTAKING -> {
        feeder.setGoal(Feeder.Goal.IDLING);
        indexer.setGoal(Indexer.Goal.STATION_INTAKING);
        intake.setGoal(Intake.Goal.IDLING);
        if (sensorInputs.shooterStaged) { // TODO: add this banner sensor
          indexer.setGoal(Indexer.Goal.IDLING);
        }
      }
      case EJECTING_TO_FLOOR -> {
        feeder.setGoal(Feeder.Goal.EJECTING);
        indexer.setGoal(Indexer.Goal.EJECTING);
        intake.setGoal(Intake.Goal.EJECTING);
      }
      case FEEDING_TO_SHOOTER -> {
        feeder.setGoal(Feeder.Goal.SHOOTING);
        indexer.setGoal(Indexer.Goal.SHOOTING);
        intake.setGoal(Intake.Goal.IDLING);
      }
    }

    feeder.periodic();
    indexer.periodic();
    intake.periodic();
  }

  public Command idle() {
    return runOnce(() -> setGoal(Goal.IDLING)).withName("Rollers Idle");
  }

  public Command floorIntake() {
    return startEnd(() -> setGoal(Goal.FLOOR_INTAKING), () -> setGoal(Goal.IDLING))
        .withName("Rollers Floor Intake");
  }

  public Command stationIntake() {
    return startEnd(() -> setGoal(Goal.STATION_INTAKING), () -> setGoal(Goal.IDLING))
        .withName("Rollers Station Intake");
  }

  public Command ejectFloor() {
    return startEnd(() -> setGoal(Goal.EJECTING_TO_FLOOR), () -> setGoal(Goal.IDLING))
        .withName("Rollers Eject Floor");
  }

  public Command feedShooter() {
    return startEnd(() -> setGoal(Goal.FEEDING_TO_SHOOTER), () -> setGoal(Goal.IDLING))
        .withName("Rollers Feed Shooter");
  }
}
