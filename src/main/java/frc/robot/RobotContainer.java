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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.kitbotshooter.*;
import frc.robot.util.trajectory.ChoreoTrajectoryReader;
import frc.robot.util.trajectory.Trajectory;
import java.io.File;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Load robot state as field
  private final RobotState robotState = RobotState.getInstance();

  // Subsystems
  private final Drive drive;
  private final KitbotShooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations\
        switch (Constants.getRobot()) {
          default -> {
            drive =
                new Drive(
                    new GyroIOPigeon2(false),
                    new ModuleIOSparkMax(DriveConstants.moduleConfigs[0]),
                    new ModuleIOSparkMax(DriveConstants.moduleConfigs[1]),
                    new ModuleIOSparkMax(DriveConstants.moduleConfigs[2]),
                    new ModuleIOSparkMax(DriveConstants.moduleConfigs[3]));
            shooter = new KitbotShooter(new KitbotFlywheelIO() {}, new KitbotFeederIO() {});
          }
        }
      }
      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.moduleConfigs[0]),
                new ModuleIOSim(DriveConstants.moduleConfigs[1]),
                new ModuleIOSim(DriveConstants.moduleConfigs[2]),
                new ModuleIOSim(DriveConstants.moduleConfigs[3]));
        shooter = new KitbotShooter(new KitbotFlywheelIOSim(), new KitbotFeederIOSim());
      }
      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new KitbotShooter(new KitbotFlywheelIO() {}, new KitbotFeederIO() {});
      }
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        Commands.sequence(
            Commands.runOnce(
                () -> shooter.setCurrentMode(KitbotShooter.Mode.CHARACTERIZING), shooter),
            new FeedForwardCharacterization(
                shooter, shooter::runFlywheelVolts, shooter::getFlywheelVelocityRadPerSec),
            Commands.runOnce(() -> shooter.setCurrentMode(null))));

    // Testing autos paths
    Function<File, Optional<Command>> trajectoryCommandFactory =
        trajectoryFile -> {
          Optional<Trajectory> trajectory = ChoreoTrajectoryReader.generate(trajectoryFile);
          return trajectory.map(
              traj ->
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              robotState.resetPose(
                                  traj.startPose(), drive.getWheelPositions(), drive.getGyroYaw()),
                          drive),
                      new DriveTrajectory(drive, traj)));
        };
    final File rootTrajectoryDir = new File(Filesystem.getDeployDirectory(), "choreo");
    for (File trajectoryFile : Objects.requireNonNull(rootTrajectoryDir.listFiles())) {
      trajectoryCommandFactory
          .apply(trajectoryFile)
          .ifPresent(
              command -> {
                autoChooser.addOption(trajectoryFile.getName(), command);
              });
    }

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
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    robotState.getEstimatedPose().getTranslation(),
                                    new Rotation2d()),
                                drive.getWheelPositions(),
                                drive.getGyroYaw()),
                    drive)
                .ignoringDisable(true));
    controller
        .button(8)
        .onTrue(
            Commands.runOnce(
                () ->
                    RobotState.getInstance()
                        .resetPose(new Pose2d(), drive.getWheelPositions(), drive.getGyroYaw())));
    controller
        .a()
        .whileTrue(
            Commands.run(() -> shooter.runFlywheelVelocity(flywheelSpeedInput.get()), shooter))
        .whileFalse(Commands.run(() -> shooter.runFlywheelVelocity(0.0), shooter));
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
