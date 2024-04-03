// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

public class RobotContainerTest {

  @Test
  public void createRobotContainer() {
    // Instantiate RobotContainer
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
      fail("Failed to instantiate RobotContainer, see stack trace above.");
    }
  }
}
