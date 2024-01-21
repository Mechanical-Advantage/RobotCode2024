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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.trajectory.ChoreoTrajectoryReader;
import frc.robot.util.trajectory.Trajectory;
import java.io.File;
import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                    Arrays.stream(DriveConstants.moduleConfigs)
                        .map(ModuleIOSparkMax::new)
                        .toArray(ModuleIO[]::new));
            shooter = new Shooter(new ShooterIOSparkMax());
          }
        }
      }
      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                Arrays.stream(DriveConstants.moduleConfigs)
                    .map(ModuleIOSim::new)
                    .toArray(ModuleIO[]::new));
        shooter = new Shooter(new ShooterIOSim());
      }
      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO[] {
                  new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
                });
        shooter = new Shooter(new ShooterIO() {});
      }
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Left Flywheels FF Characterization",
        new FeedForwardCharacterization(
                shooter,
                shooter::runLeftCharacterizationVolts,
                shooter::getLeftCharacterizationVelocity)
            .beforeStarting(() -> shooter.setCharacterizing(true))
            .finallyDo(() -> shooter.setCharacterizing(false)));
    autoChooser.addOption(
        "Right Flywheels FF Characterization",
        new FeedForwardCharacterization(
                shooter,
                shooter::runRightCharacterizationVolts,
                shooter::getRightCharacterizationVelocity)
            .beforeStarting(() -> shooter.setCharacterizing(true))
            .finallyDo(() -> shooter.setCharacterizing(false)));

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
                                  AllianceFlipUtil.apply(traj.startPose()),
                                  drive.getWheelPositions(),
                                  drive.getGyroYaw()),
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
                                    AllianceFlipUtil.apply(new Rotation2d())),
                                drive.getWheelPositions(),
                                drive.getGyroYaw()),
                    drive)
                .ignoringDisable(true));
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () ->
                    RobotState.getInstance()
                        .resetPose(new Pose2d(), drive.getWheelPositions(), drive.getGyroYaw())));
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
