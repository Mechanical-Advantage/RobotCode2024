// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2024.subsystems.superstructure.GenericSlamElevatorIOSim;

public class BackpackActuatorIOSim extends GenericSlamElevatorIOSim implements BackpackActuatorIO {
  private static final double maxLengthMeters = Units.inchesToMeters(11.872);
  private static final double reduction = 32.0 / 10.0;
  private static final double drumRadiusMeters = Units.inchesToMeters(0.5);

  public BackpackActuatorIOSim() {
    super(maxLengthMeters, reduction, drumRadiusMeters);
  }
}
