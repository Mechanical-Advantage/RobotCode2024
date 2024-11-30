// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class BrakeModeController {
  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecThreshold =
      new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", 0.05);
  private final Timer lastMovementTimer = new Timer();
  private Drive.CoastRequest coastRequest = Drive.CoastRequest.AUTOMATIC;
  private boolean lastEnabled = false;

  @AutoLogOutput(key = "Drive/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  public void updateBrakeMode(Module[] modules) {
    if (Arrays.stream(modules)
        .anyMatch(
            module ->
                Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecThreshold.get())) {
      lastMovementTimer.reset();
    }
    if (DriverStation.isEnabled() && !lastEnabled) {
      coastRequest = Drive.CoastRequest.AUTOMATIC;
    }
    lastEnabled = DriverStation.isEnabled();
    switch (coastRequest) {
      case AUTOMATIC:
        if (DriverStation.isEnabled()) {
          setBrakeMode(true, modules);
        } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
          setBrakeMode(false, modules);
        }
        break;
      case ALWAYS_BRAKE:
        setBrakeMode(true, modules);
        break;
      case ALWAYS_COAST:
        setBrakeMode(false, modules);
        break;
    }
  }

  public void setBrakeModeEnabled() {
    brakeModeEnabled = true;
  }

  private void setBrakeMode(boolean enabled, Module[] modules) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }
}
