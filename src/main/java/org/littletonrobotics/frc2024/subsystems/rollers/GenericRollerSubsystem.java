package org.littletonrobotics.frc2024.subsystems.rollers;

import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public abstract class GenericRollerSubsystem<G extends GenericRollerSubsystem.VoltageGoal> {
  protected interface VoltageGoal {
    DoubleSupplier getVoltageSupplier();
  }

  public abstract G getGoal();

  private final String name;
  private final GenericRollerSystemIO io;
  private final GenericRollerSystemIOInputsAutoLogged inputs =
      new GenericRollerSystemIOInputsAutoLogged();

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    io.runVolts(getGoal().getVoltageSupplier().getAsDouble());
    Logger.recordOutput(name + "/Goal", getGoal().toString());
  }
}
