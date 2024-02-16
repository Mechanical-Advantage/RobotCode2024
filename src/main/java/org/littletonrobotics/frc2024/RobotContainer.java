// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.frc2024.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2024.commands.auto.AutoCommands;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionConstants;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import org.littletonrobotics.frc2024.subsystems.drive.*;
import org.littletonrobotics.frc2024.subsystems.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelsIO;
import org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelsIOSim;
import org.littletonrobotics.frc2024.subsystems.flywheels.FlywheelsIOSparkFlex;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.rollers.RollersSensorsIO;
import org.littletonrobotics.frc2024.subsystems.rollers.RollersSensorsIOReal;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIO;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIOSim;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.Indexer;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIO;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIOSim;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIOSparkFlex;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIO;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOSim;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
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
  private Flywheels flywheels;
  private Rollers rollers;
  private Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Declare component subsystems (not visible outside constructor)
    Feeder feeder = null;
    Indexer indexer = null;
    Intake intake = null;
    Arm arm = null;

    // Create subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          // No impl yet
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(false),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[0]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[1]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[2]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[3]));
          aprilTagVision =
              new AprilTagVision(
                  new AprilTagVisionIONorthstar(
                      AprilTagVisionConstants.instanceNames[0],
                      AprilTagVisionConstants.cameraIds[0]),
                  new AprilTagVisionIONorthstar(
                      AprilTagVisionConstants.instanceNames[1],
                      AprilTagVisionConstants.cameraIds[1]));
          flywheels = new Flywheels(new FlywheelsIOSparkFlex());

          feeder = new Feeder(new FeederIOKrakenFOC());
          indexer = new Indexer(new IndexerIOSparkFlex());
          intake = new Intake(new IntakeIOKrakenFOC());
          rollers = new Rollers(feeder, indexer, intake, new RollersSensorsIOReal());

          arm = new Arm(new ArmIOKrakenFOC());
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.moduleConfigs[0]),
                  new ModuleIOSim(DriveConstants.moduleConfigs[1]),
                  new ModuleIOSim(DriveConstants.moduleConfigs[2]),
                  new ModuleIOSim(DriveConstants.moduleConfigs[3]));
          flywheels = new Flywheels(new FlywheelsIOSim());

          feeder = new Feeder(new FeederIOSim());
          indexer = new Indexer(new IndexerIOSim());
          intake = new Intake(new IntakeIOSim());
          rollers = new Rollers(feeder, indexer, intake, new RollersSensorsIO() {});

          arm = new Arm(new ArmIOSim());
        }
      }
    }

    // No-op implementation for replay
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
      switch (Constants.getRobot()) {
        case DEVBOT ->
            aprilTagVision =
                new AprilTagVision(new AprilTagVisionIO() {}, new AprilTagVisionIO() {});
        default -> aprilTagVision = new AprilTagVision();
      }
    }
    if (flywheels == null) {
      flywheels = new Flywheels(new FlywheelsIO() {});
    }
    if (feeder == null) {
      feeder = new Feeder(new FeederIO() {});
    }
    if (indexer == null) {
      indexer = new Indexer(new IndexerIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (rollers == null) {
      rollers = new Rollers(feeder, indexer, intake, new RollersSensorsIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    superstructure = new Superstructure(arm);

    // Configure autos and buttons
    configureAutos();
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    AutoCommands autoCommands = new AutoCommands(drive, superstructure);
    autoChooser.addOption("Drive Straight", autoCommands.driveStraight());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
                drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity)
            .finallyDo(drive::endCharacterization));
    autoChooser.addOption(
        "Flywheels FF Characterization",
        new FeedForwardCharacterization(
            flywheels,
            flywheels::runCharacterizationVolts,
            flywheels::getCharacterizationVelocity));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive Commands
    drive.setDefaultCommand(
        drive
            .run(
                () ->
                    drive.acceptTeleopInput(
                        -controller.getLeftY(), -controller.getLeftX(), -controller.getRightX()))
            .withName("Drive Teleop Input"));

    // Aim and Rev Flywheels
    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                    () ->
                        drive.setHeadingGoal(
                            () -> RobotState.getInstance().getAimingParameters().driveHeading()),
                    drive::clearHeadingGoal)
                .alongWith(superstructure.aim(), flywheels.shoot())
                .withName("Prepare Shot"));
    // Shoot
    Trigger readyToShoot =
        new Trigger(() -> drive.atHeadingGoal() && superstructure.atGoal() && flywheels.atGoal());
    readyToShoot
        .whileTrue(
            Commands.run(
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
        .whileFalse(
            Commands.run(
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));
    controller
        .rightTrigger()
        .and(readyToShoot)
        .onTrue(
            Commands.parallel(
                    Commands.waitSeconds(0.5),
                    Commands.waitUntil(controller.rightTrigger().negate()))
                .deadlineWith(rollers.feedShooter(), superstructure.aim(), flywheels.shoot()));
    // Intake Floor
    controller
        .leftTrigger()
        .whileTrue(
            superstructure
                .intake()
                .alongWith(
                    Commands.waitUntil(superstructure::atGoal).andThen(rollers.floorIntake()))
                .withName("Floor Intake"));
    // Eject Floor
    controller
        .leftBumper()
        .whileTrue(
            superstructure
                .intake()
                .alongWith(Commands.waitUntil(superstructure::atGoal).andThen(rollers.ejectFloor()))
                .withName("Eject To Floor"));

    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState.resetPose(
                            AllianceFlipUtil.apply(
                                new Pose2d(
                                    Units.inchesToMeters(36.0),
                                    FieldConstants.Speaker.centerSpeakerOpening.getY(),
                                    new Rotation2d()))))
                .ignoringDisable(true));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState.resetPose(
                            new Pose2d(
                                robotState.getEstimatedPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d()))))
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
