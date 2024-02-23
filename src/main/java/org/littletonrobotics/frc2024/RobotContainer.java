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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.frc2024.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2024.commands.StaticCharacterization;
import org.littletonrobotics.frc2024.commands.WheelRadiusCharacterization;
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
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.Backpack;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.BackpackIO;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.BackpackIOSim;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.BackpackIOSparkFlex;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIO;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIOSim;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.Indexer;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIO;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIODevbot;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIOSim;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIO;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOSim;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.frc2024.util.Alert.AlertType;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.OverrideSwitches;
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
  private final Rollers rollers;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger autoAimDisable = overrides.operatorSwitch(0);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.INFO);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Declare component subsystems (not visible outside constructor)
    Feeder feeder = null;
    Indexer indexer = null;
    Intake intake = null;
    Backpack backpack = null;
    Arm arm = null;
    RollersSensorsIO rollersSensorsIO = null;

    // Create subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(true),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3]));
          arm = new Arm(new ArmIOKrakenFOC());
          intake = new Intake(new IntakeIOKrakenFOC());
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
          indexer = new Indexer(new IndexerIODevbot());
          intake = new Intake(new IntakeIOKrakenFOC());
          backpack = new Backpack(new BackpackIOSparkFlex());
          rollersSensorsIO = new RollersSensorsIOReal();

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
          backpack = new Backpack(new BackpackIOSim());
          rollersSensorsIO = new RollersSensorsIO() {};

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
    if (backpack == null) {
      backpack = new Backpack(new BackpackIO() {});
    }
    if (rollersSensorsIO == null) {
      rollersSensorsIO = new RollersSensorsIO() {};
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    rollers = new Rollers(feeder, indexer, intake, backpack, rollersSensorsIO);
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
                drive, drive::runCharacterization, drive::getCharacterizationVelocity)
            .finallyDo(drive::endCharacterization));
    autoChooser.addOption(
        "Flywheels FF Characterization",
        new FeedForwardCharacterization(
            flywheels, flywheels::runCharacterization, flywheels::getCharacterizationVelocity));
    autoChooser.addOption(
        "Drive Static Characterization",
        new StaticCharacterization(
            drive, drive::runCharacterization, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheels Static Characterization",
        new StaticCharacterization(
            flywheels, flywheels::runCharacterization, flywheels::getCharacterizationVelocity));
    autoChooser.addOption(
        "Arm Static Characterization",
        new StaticCharacterization(
                superstructure,
                superstructure::runArmCharacterization,
                superstructure::getArmCharacterizationVelocity)
            .finallyDo(superstructure::endArmCharacterization));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        drive
            .orientModules(Drive.getCircleOrientations())
            .andThen(
                new WheelRadiusCharacterization(
                    drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE))
            .withName("Drive Wheel Radius Characterization"));
    autoChooser.addOption("Diagnose Arm", superstructure.diagnoseArm());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive commands
    drive.setDefaultCommand(
        drive
            .run(
                () ->
                    drive.acceptTeleopInput(
                        -controller.getLeftY(), -controller.getLeftX(), -controller.getRightX()))
            .withName("Drive Teleop Input"));

    // Aim and rev flywheels
    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                    () ->
                        drive.setHeadingGoal(
                            () -> RobotState.getInstance().getAimingParameters().driveHeading()),
                    drive::clearHeadingGoal)
                .alongWith(
                    Commands.either(
                        superstructure.subwoofer(), superstructure.aim(), autoAimDisable),
                    flywheels.shootCommand())
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
                .deadlineWith(
                    rollers.feedShooter(), superstructure.aim(), flywheels.shootCommand()));

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

    // Amp scoring
    controller
        .rightBumper()
        .whileTrue(
            superstructure
                .amp()
                .alongWith(
                    Commands.startEnd(
                        () -> drive.setHeadingGoal(() -> new Rotation2d(-Math.PI / 2.0)),
                        drive::clearHeadingGoal)));
    controller
        .rightBumper()
        .and(controller.rightTrigger())
        .whileTrue(Commands.waitUntil(superstructure::atGoal).andThen(rollers.ampScore()));

    // Shot compensation adjustment
    controller
        .povUp()
        .onTrue(
            Commands.runOnce(() -> RobotState.getInstance().adjustShotCompensationDegrees(0.1)));
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(() -> RobotState.getInstance().adjustShotCompensationDegrees(-0.1)));

    // Reset pose
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

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(controller.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(controller.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());
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
