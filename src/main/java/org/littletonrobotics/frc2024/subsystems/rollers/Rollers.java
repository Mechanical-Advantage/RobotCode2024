// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.Backpack;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.Indexer;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.frc2024.util.NoteVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private final Feeder feeder;
  private final Indexer indexer;
  private final Intake intake;
  private final Backpack backpack;

  private final RollersSensorsIO sensorsIO;
  private final RollersSensorsIOInputsAutoLogged sensorInputs =
      new RollersSensorsIOInputsAutoLogged();

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    EJECT_TO_FLOOR,
    QUICK_INTAKE_TO_FEED,
    FEED_TO_SHOOTER,
    AMP_SCORE,
    SHUFFLE_BACKPACK,
    SHUFFLE_SHOOTER
  }

  public enum GamepieceState {
    NONE,
    SHOOTER_STAGED,
    BACKPACK_STAGED
  }

  @Getter private Goal goal = Goal.IDLE;
  @AutoLogOutput @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NONE;
  private GamepieceState lastGamepieceState = GamepieceState.NONE;
  private Timer gamepieceStateTimer = new Timer();

  @Setter private BooleanSupplier backpackActuatedSupplier = () -> false;

  public Rollers(
      Feeder feeder,
      Indexer indexer,
      Intake intake,
      Backpack backpack,
      RollersSensorsIO sensorsIO) {
    this.feeder = feeder;
    this.indexer = indexer;
    this.intake = intake;
    this.backpack = backpack;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(setGoalCommand(Goal.IDLE));
    gamepieceStateTimer.start();
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;
    }

    if (sensorInputs.shooterStaged) {
      gamepieceState = GamepieceState.SHOOTER_STAGED;
    } else if (sensorInputs.backbackStaged) {
      gamepieceState = GamepieceState.BACKPACK_STAGED;
    } else {
      gamepieceState = GamepieceState.NONE;
    }
    if (gamepieceState != lastGamepieceState) {
      gamepieceStateTimer.reset();
    }
    lastGamepieceState = gamepieceState;

    NoteVisualizer.setHasNote(gamepieceState != GamepieceState.NONE);

    // Reset idle and wait for other input
    feeder.setGoal(Feeder.Goal.IDLING);
    indexer.setGoal(Indexer.Goal.IDLING);
    intake.setGoal(Intake.Goal.IDLING);
    backpack.setGoal(Backpack.Goal.IDLING);
    switch (goal) {
      case IDLE -> {}
      case FLOOR_INTAKE -> {
        feeder.setGoal(Feeder.Goal.FLOOR_INTAKING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
        if (gamepieceState == GamepieceState.SHOOTER_STAGED) {
          indexer.setGoal(Indexer.Goal.IDLING);
        } else {
          indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
        }
      }
      case STATION_INTAKE -> {
        if (gamepieceState != GamepieceState.NONE && gamepieceStateTimer.hasElapsed(0.2)) {
          indexer.setGoal(Indexer.Goal.IDLING);
        } else {
          indexer.setGoal(Indexer.Goal.STATION_INTAKING);
        }
      }
      case EJECT_TO_FLOOR -> {
        feeder.setGoal(Feeder.Goal.EJECTING);
        indexer.setGoal(Indexer.Goal.EJECTING);
        intake.setGoal(Intake.Goal.EJECTING);
        backpack.setGoal(Backpack.Goal.EJECTING);
      }
      case QUICK_INTAKE_TO_FEED -> {
        feeder.setGoal(Feeder.Goal.SHOOTING);
        indexer.setGoal(Indexer.Goal.SHOOTING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
      }
      case FEED_TO_SHOOTER -> {
        feeder.setGoal(Feeder.Goal.SHOOTING);
        indexer.setGoal(Indexer.Goal.SHOOTING);
      }
      case AMP_SCORE -> {
        feeder.setGoal(Feeder.Goal.SHUFFLING);
        indexer.setGoal(Indexer.Goal.EJECTING);
        backpack.setGoal(Backpack.Goal.AMP_SCORING);
      }
      case SHUFFLE_BACKPACK -> {
        // Shuffle into backpack
        feeder.setGoal(Feeder.Goal.SHUFFLING);
        indexer.setGoal(Indexer.Goal.EJECTING);
        if (gamepieceState != GamepieceState.BACKPACK_STAGED) {
          backpack.setGoal(Backpack.Goal.AMP_SCORING);
        } else {
          backpack.setGoal(Backpack.Goal.IDLING);
        }
      }
      case SHUFFLE_SHOOTER -> {
        // Shuffle into shooter
        feeder.setGoal(Feeder.Goal.SHUFFLING);
        backpack.setGoal(Backpack.Goal.EJECTING);
        if (gamepieceState != GamepieceState.SHOOTER_STAGED) {
          indexer.setGoal(Indexer.Goal.FLOOR_INTAKING);
        } else {
          indexer.setGoal(Indexer.Goal.IDLING);
        }
      }
    }

    feeder.periodic();
    indexer.periodic();
    intake.periodic();
    backpack.periodic();
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> this.goal = goal, () -> this.goal = Goal.IDLE)
        .withName("Rollers " + goal);
  }

  public Command shuffle() {
    return Commands.either(
        setGoalCommand(Goal.SHUFFLE_BACKPACK)
            .until(() -> gamepieceState == GamepieceState.BACKPACK_STAGED),
        setGoalCommand(Goal.SHUFFLE_SHOOTER)
            .until(() -> gamepieceState == GamepieceState.SHOOTER_STAGED),
        () -> gamepieceState == GamepieceState.SHOOTER_STAGED);
  }
}
