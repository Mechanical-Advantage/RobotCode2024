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

  private boolean intake = false;
  private boolean eject = false;

  public Intake(IntakeIO io) {
    System.out.println("[Init] Creating Intake");
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
    } else {
      if (intake) {
        io.setVoltage(intakeVoltage.get());
      } else if (eject) {
        io.setVoltage(-intakeVoltage.get());
      }
    }
  }

  public boolean intaking() {
    return intake;
  }

  public boolean ejecting() {
    return eject;
  }

  public boolean running() {
    return eject || intake;
  }

  private void intake() {
    intake = true;
    eject = false;
  }

  private void eject() {
    eject = true;
    intake = false;
  }

  private void stop() {
    intake = false;
    eject = false;
    io.stop();
  }

  public Command intakeCommand() {
    return Commands.runOnce(this::intake);
  }

  public Command ejectCommand() {
    return Commands.runOnce(this::eject);
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }
}
