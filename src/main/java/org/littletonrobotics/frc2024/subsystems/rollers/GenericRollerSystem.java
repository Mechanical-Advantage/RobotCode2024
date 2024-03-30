// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers;

import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2024.Robot;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public abstract class GenericRollerSystem<G extends GenericRollerSystem.VoltageGoal> {
  public interface VoltageGoal {
    DoubleSupplier getVoltageSupplier();
  }

  public abstract G getGoal();

  private final String name;
  private final GenericRollerSystemIO io;
  private final GenericRollerSystemIOInputsAutoLogged inputs =
      new GenericRollerSystemIOInputsAutoLogged();
  private final Alert disconnected;

  public GenericRollerSystem(String name, GenericRollerSystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.WARNING);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    Robot.totalCurrent += inputs.supplyCurrentAmps;
    disconnected.set(!inputs.connected);

    io.runVolts(getGoal().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }
}
