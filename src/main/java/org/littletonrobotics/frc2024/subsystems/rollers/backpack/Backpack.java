// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers.backpack;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystem;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;

@Setter
@Getter
public class Backpack extends GenericRollerSystem<Backpack.Goal> {

  @RequiredArgsConstructor
  @Getter
  public enum Goal implements GenericRollerSystem.VoltageGoal {
    IDLING(VoltageSupplierFactory.getIdleVoltage()),
    AMP_SCORING(VoltageSupplierFactory.getAmpScoringVoltage()),
    TRAP_SCORING(VoltageSupplierFactory.getTrapScoringVoltage()),
    TRAP_JACKHAMMER_OUT(VoltageSupplierFactory.getJackhammerOutVoltage()),
    TRAP_JACKHAMMER_IN(VoltageSupplierFactory.getJackhammerInVoltage()),
    EJECTING(VoltageSupplierFactory.getEjectingVoltage());

    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.IDLING;

  public Backpack(BackpackIO io) {
    super("Backpack", io);
  }

  // Utility class for managing voltage suppliers
  private static class VoltageSupplierFactory {
    static DoubleSupplier getIdleVoltage() {
      return () -> 0;
    }

    static DoubleSupplier getAmpScoringVoltage() {
      return new LoggedTunableNumber("Backpack/AmpScoringVoltage", 12.0);
    }

    static DoubleSupplier getTrapScoringVoltage() {
      return new LoggedTunableNumber("Backpack/TrapScoringVoltage", 8.0);
    }

    static DoubleSupplier getJackhammerOutVoltage() {
      return new LoggedTunableNumber("Backpack/JackhammerOutVoltage", 8.0);
    }

    static DoubleSupplier getJackhammerInVoltage() {
      return new LoggedTunableNumber("Backpack/JackhammerInVoltage", -2.0);
    }

    static DoubleSupplier getEjectingVoltage() {
      return new LoggedTunableNumber("Backpack/EjectingVoltage", -12.0);
    }
  }
}
