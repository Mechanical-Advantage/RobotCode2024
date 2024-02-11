package org.littletonrobotics.frc2024.subsystems.rollers.intake;

import org.littletonrobotics.frc2024.subsystems.rollers.GenericRollerSystemIOKrakenFOC;

public class IntakeIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements IntakeIO {
  private static final int id = 2;
  private static final String bus = "canivore";
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final boolean brake = false;
  private static final double reduction = 18.0 / 12.0;

  public IntakeIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }
}
