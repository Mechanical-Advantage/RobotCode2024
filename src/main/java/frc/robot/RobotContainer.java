// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private Flywheel shooter;
  private Flywheel hopper;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private static final double CONTROLLER_DEADBAND = 0.09;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelVoltInput =
      new LoggedDashboardNumber("Flywheel Speed", 12.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOTalonSRX());
        hopper = new Flywheel(new FlywheelIOTalonSRX(14));
        shooter = new Flywheel(new FlywheelIOTalonSRX(15));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim());
        shooter = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {});
        shooter = new Flywheel(new FlywheelIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(() -> shooter.runVolts(12.0), shooter::stop, shooter).withTimeout(5.0));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization Left",
        new FeedForwardCharacterization(
            drive, (volts) -> drive.driveVolts(volts, volts), drive::getLeftVelocityMetersPerSec));
    autoChooser.addOption(
        "Drive FF Characterization Right",
        new FeedForwardCharacterization(
            drive, (volts) -> drive.driveVolts(volts, volts), drive::getRightVelocityMetersPerSec));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            shooter, shooter::runVolts, shooter::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        Commands.run(
            () ->
                drive.driveArcade(
                    (controller.getLeftY() < CONTROLLER_DEADBAND
                            && controller.getLeftY() > -CONTROLLER_DEADBAND)
                        ? 0
                        : -controller.getLeftY(),
                    (controller.getRightX() < CONTROLLER_DEADBAND
                            && controller.getRightX() > -CONTROLLER_DEADBAND)
                        ? 0
                        : -1 * controller.getRightX()),
            drive));
    controller
        .a()
        .whileTrue(
            new PrepareLaunch(shooter, hopper)
                .withTimeout(Constants.SHOOTER_DELAY)
                .andThen(new LaunchNote(shooter, hopper))
                .handleInterrupt(() -> shooter.stop()));
    controller.b().whileTrue(new IntakeNote(shooter, hopper).handleInterrupt(() -> shooter.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
