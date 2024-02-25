// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import java.time.Duration;

public class RollersSensorsIOCompbot implements RollersSensorsIO {
  private final DigitalInput shooterStagedSensor = new DigitalInput(0);
  private final DigitalInput backpackStagedSensor = new DigitalInput(1);
  private final DigitalGlitchFilter glitchFilter = new DigitalGlitchFilter();

  public RollersSensorsIOCompbot() {
    glitchFilter.setPeriodNanoSeconds(Duration.ofMillis(5).toNanos());
    glitchFilter.add(shooterStagedSensor);
    glitchFilter.add(backpackStagedSensor);
  }

  @Override
  public void updateInputs(RollersSensorsIOInputs inputs) {
    inputs.shooterStaged = shooterStagedSensor.get();
    inputs.backbackStaged = backpackStagedSensor.get();
  }
}
