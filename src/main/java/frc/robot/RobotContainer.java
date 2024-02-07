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
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.superstructure.Arm.Arm;
import frc.robot.subsystems.superstructure.Arm.ArmIO;
import frc.robot.subsystems.superstructure.Arm.ArmIOKrakenFOC;
import frc.robot.subsystems.superstructure.Arm.ArmIOSim;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeIO;
import frc.robot.subsystems.superstructure.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIO;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSparkMax;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.trajectory.ChoreoTrajectoryReader;
import frc.robot.util.trajectory.HolonomicTrajectory;
import java.io.File;
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
  // Load robot state
  private final RobotState robotState = RobotState.getInstance();

  // Subsystems
  private Drive drive;
  private AprilTagVision aprilTagVision;
  private Shooter shooter;
  private Intake intake;
  private Arm arm;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case DEVBOT -> {
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(DriveConstants.moduleConfigs[0]),
                new ModuleIOSparkMax(DriveConstants.moduleConfigs[1]),
                new ModuleIOSparkMax(DriveConstants.moduleConfigs[2]),
                new ModuleIOSparkMax(DriveConstants.moduleConfigs[3]));
        arm = new Arm(new ArmIOKrakenFOC());
        shooter = new Shooter(new ShooterIOSparkMax());
      }
      case SIMBOT -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.moduleConfigs[0]),
                new ModuleIOSim(DriveConstants.moduleConfigs[1]),
                new ModuleIOSim(DriveConstants.moduleConfigs[2]),
                new ModuleIOSim(DriveConstants.moduleConfigs[3]));
        arm = new Arm(new ArmIOSim());
        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new IntakeIOSim());
      }
      case COMPBOT -> {
        // No impl yet
      }
    }

    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    if (aprilTagVision == null) {
      aprilTagVision = new AprilTagVision();
    }

    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }

    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }

    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
                drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity)
            .finallyDo(drive::endCharacterization));
    autoChooser.addOption(
        "Left Flywheels FF Characterization",
        new FeedForwardCharacterization(
            shooter,
            shooter::runLeftCharacterizationVolts,
            shooter::getLeftCharacterizationVelocity));
    autoChooser.addOption(
        "Right Flywheels FF Characterization",
        new FeedForwardCharacterization(
            shooter,
            shooter::runRightCharacterizationVolts,
            shooter::getRightCharacterizationVelocity));
    autoChooser.addOption("Arm get static current", arm.getStaticCurrent());

    // Testing autos paths
    Function<File, Optional<Command>> trajectoryCommandFactory =
        trajectoryFile -> {
          Optional<HolonomicTrajectory> trajectory =
              ChoreoTrajectoryReader.generate(trajectoryFile);
          return trajectory.map(
              traj ->
                  Commands.runOnce(
                          () -> robotState.resetPose(AllianceFlipUtil.apply(traj.getStartPose())))
                      .andThen(Commands.runOnce(() -> drive.setTrajectoryGoal(traj)))
                      .until(drive::isTrajectoryGoalCompleted));
        };
    final File rootTrajectoryDir = new File(Filesystem.getDeployDirectory(), "choreo");
    for (File trajectoryFile : Objects.requireNonNull(rootTrajectoryDir.listFiles())) {
      trajectoryCommandFactory
          .apply(trajectoryFile)
          .ifPresent(command -> autoChooser.addOption(trajectoryFile.getName(), command));
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
        drive.run(
            () ->
                drive.setTeleopDriveGoal(
                    -controller.getLeftY(), -controller.getLeftX(), -controller.getRightX())));
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.setAutoAlignGoal(
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                FieldConstants.ampCenter.getX(),
                                FieldConstants.ampCenter.getY() - 0.6,
                                new Rotation2d(Math.PI / 2.0))))))
        .onFalse(Commands.runOnce(drive::clearAutoAlignGoal));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState.resetPose(
                            new Pose2d(
                                robotState.getEstimatedPose().getTranslation(), new Rotation2d())))
                .ignoringDisable(true));
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
