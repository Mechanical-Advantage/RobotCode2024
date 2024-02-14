// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.GyroIO;

public class ResetGyro extends Command {
  GyroIO gyro;

  // CANLauncher m_launcher;

  /** Creates a new LaunchNote. */
  public ResetGyro(GyroIO gyro) {
    this.gyro = gyro;
  }

  @Override
  public void initialize() {
    gyro.resetGyro();
  }
}
