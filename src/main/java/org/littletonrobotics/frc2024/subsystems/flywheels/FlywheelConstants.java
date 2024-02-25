// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.flywheels;

import org.littletonrobotics.frc2024.Constants;

public class FlywheelConstants {
  public static final FlywheelConfig flywheelConfig =
      switch (Constants.getRobot()) {
        case COMPBOT -> new FlywheelConfig(4, 0, (1.0 / 2.0), 12000.0);
        case DEVBOT -> new FlywheelConfig(5, 4, (1.0 / 2.0), 6000.0);
        case SIMBOT -> new FlywheelConfig(0, 0, (1.0 / 2.0), 6000.0);
      };

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case COMPBOT -> new Gains(6, 0, 0, 35, 0, 0);
        case DEVBOT -> new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
      };

  public record FlywheelConfig(
      int leftID, int rightID, double reduction, double maxAcclerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
