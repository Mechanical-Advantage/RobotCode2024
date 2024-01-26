package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Intake extends SubsystemBase {
  private final LoggedDashboardNumber intakeVoltage =
      new LoggedDashboardNumber("Intake/intakeVoltage", 12.0);
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean run = false;

  public Intake(IntakeIO io) {
    this.io = io;
    // TODO: test if this is needed
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      run = false;
    } else {
      if (run) {
        io.setVoltage(intakeVoltage.get());
      }
    }
  }

  private void run() {
    run = true;
  }

  private void stop() {
    run = false;
    io.stop();
  }

  public Command runCommand() {
    return Commands.runOnce(this::run);
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }
}
