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

  public Rollers(Feeder feeder, Indexer indexer, Intake intake, RollersSensorsIO sensorsIO) {
    this.feeder = feeder;
    this.indexer = indexer;
    this.intake = intake;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(runOnce(this::goIdle).withName("RollersIdle"));
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    if (DriverStation.isDisabled()) {
      goIdle();
    }

    feeder.periodic();
    indexer.periodic();
    intake.periodic();
  }

  private void goIdle() {
    feeder.setGoal(Feeder.Goal.IDLE);
    indexer.setGoal(Indexer.Goal.IDLE);
    intake.setGoal(Intake.Goal.IDLE);
  }

  public Command floorIntakeCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.FLOOR_INTAKING);
              indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
              intake.setGoal(Intake.Goal.FLOOR_INTAKING);
              if (sensorInputs.shooterStaged) {
                indexer.setGoal(Indexer.Goal.IDLE);
              }
            },
            this::goIdle)
        .withName("RollersFloorIntake");
  }

  public Command stationIntakeCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.IDLE);
              indexer.setGoal(Indexer.Goal.STATION_INTAKING);
              intake.setGoal(Intake.Goal.IDLE);
              if (sensorInputs.shooterStaged) { // TODO: ADD THIS BANNER
                indexer.setGoal(Indexer.Goal.IDLE);
              }
            },
            this::goIdle)
        .withName("RollersStationIntake");
  }

  public Command ejectFloorCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.EJECTING);
              indexer.setGoal(Indexer.Goal.EJECTING);
              intake.setGoal(Intake.Goal.EJECTING);
            },
            this::goIdle)
        .withName("RollersEjectFloor");
  }

  public Command feedShooterCommand() {
    return startEnd(
            () -> {
              feeder.setGoal(Feeder.Goal.SHOOTING);
              indexer.setGoal(Indexer.Goal.SHOOTING);
              intake.setGoal(Intake.Goal.IDLE);
            },
            this::goIdle)
        .withName("RollersFeedShooter");
  }
}
