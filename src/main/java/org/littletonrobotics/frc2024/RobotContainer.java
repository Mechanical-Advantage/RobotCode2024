// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.AutoSelector.AutoQuestion;
import org.littletonrobotics.frc2024.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2024.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2024.commands.ClimbingCommands;
import org.littletonrobotics.frc2024.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2024.commands.StaticCharacterization;
import org.littletonrobotics.frc2024.commands.WheelRadiusCharacterization;
import org.littletonrobotics.frc2024.commands.auto.AutoBuilder;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import org.littletonrobotics.frc2024.subsystems.drive.*;
import org.littletonrobotics.frc2024.subsystems.drive.controllers.TeleopDriveController;
import org.littletonrobotics.frc2024.subsystems.flywheels.*;
import org.littletonrobotics.frc2024.subsystems.leds.Leds;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers.GamepieceState;
import org.littletonrobotics.frc2024.subsystems.rollers.RollersSensorsIO;
import org.littletonrobotics.frc2024.subsystems.rollers.RollersSensorsIOCompbot;
import org.littletonrobotics.frc2024.subsystems.rollers.RollersSensorsIODevbot;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.Backpack;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.BackpackIO;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.BackpackIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.BackpackIOSim;
import org.littletonrobotics.frc2024.subsystems.rollers.backpack.BackpackIOSparkFlex;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIO;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIOSim;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.*;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIO;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator.BackpackActuator;
import org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator.BackpackActuatorIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator.BackpackActuatorIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.backpackactuator.BackpackActuatorIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.climber.Climber;
import org.littletonrobotics.frc2024.subsystems.superstructure.climber.ClimberIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.climber.ClimberIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.climber.ClimberIOSim;
import org.littletonrobotics.frc2024.util.*;
import org.littletonrobotics.frc2024.util.Alert.AlertType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({DoublePressTracker.class})
public class RobotContainer {
  // Load static objects
  private final RobotState robotState = RobotState.getInstance();
  private final Leds leds = Leds.getInstance();

  // Subsystems
  private Drive drive;
  private AprilTagVision aprilTagVision;
  private Flywheels flywheels;
  private final Rollers rollers;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger robotRelative = overrides.driverSwitch(0);
  private final Trigger armDisable = overrides.driverSwitch(1);
  private final Trigger armCoast = overrides.driverSwitch(2);
  private final Trigger aprilTagsSpeakerOnly = overrides.multiDirectionSwitchLeft();
  private final Trigger aprilTagsAmpOnly = overrides.multiDirectionSwitchRight();
  private final Trigger shootPresets = overrides.operatorSwitch(0);
  private final Trigger shootAlignDisable = overrides.operatorSwitch(1);
  private final Trigger lookaheadDisable = overrides.operatorSwitch(2);
  private final Trigger autoDriveDisable = overrides.operatorSwitch(3);
  private final Trigger autoFlywheelSpinupDisable = overrides.operatorSwitch(4);
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);
  private final Alert ridiculousAutoAlert =
      new Alert("The selected auto is ridiculous! 😡", AlertType.WARNING);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.INFO);
  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 15.0);

  private boolean podiumShotMode = false;
  private boolean armCoastOverride = false;

  // Dashboard inputs
  private final AutoSelector autoSelector = new AutoSelector("Auto");

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getAprilTagLayoutType() {
    if (aprilTagsSpeakerOnly.getAsBoolean()) {
      return FieldConstants.AprilTagLayoutType.SPEAKERS_ONLY;
    } else if (aprilTagsAmpOnly.getAsBoolean()) {
      return FieldConstants.AprilTagLayoutType.AMPS_ONLY;
    } else {
      return FieldConstants.defaultAprilTagType;
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Declare component subsystems (not visible outside constructor)
    Feeder feeder = null;
    Indexer indexer = null;
    Intake intake = null;
    Backpack backpack = null;
    Arm arm = null;
    Climber climber = null;
    BackpackActuator backpackActuator = null;
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
          aprilTagVision =
              new AprilTagVision(
                  this::getAprilTagLayoutType,
                  new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 0),
                  new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 1),
                  new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 2),
                  new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 3));
          flywheels = new Flywheels(new FlywheelsIOKrakenFOC());
          feeder = new Feeder(new FeederIOKrakenFOC());
          indexer = new Indexer(new IndexerIOCompbot());
          intake = new Intake(new IntakeIOKrakenFOC());
          backpack = new Backpack(new BackpackIOKrakenFOC());
          rollersSensorsIO = new RollersSensorsIOCompbot();
          arm = new Arm(new ArmIOKrakenFOC());
          climber = new Climber(new ClimberIOKrakenFOC());
          backpackActuator = new BackpackActuator(new BackpackActuatorIOKrakenFOC());
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
                  this::getAprilTagLayoutType,
                  new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 0),
                  new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 1));
          flywheels = new Flywheels(new FlywheelsIOSparkFlex());
          feeder = new Feeder(new FeederIOKrakenFOC());
          indexer = new Indexer(new IndexerIODevbot());
          intake = new Intake(new IntakeIOKrakenFOC());
          backpack = new Backpack(new BackpackIOSparkFlex());
          rollersSensorsIO = new RollersSensorsIODevbot();
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
          climber = new Climber(new ClimberIOSim());
          backpackActuator = new BackpackActuator(new BackpackActuatorIOSim());
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
        case COMPBOT ->
            aprilTagVision =
                new AprilTagVision(
                    this::getAprilTagLayoutType,
                    new AprilTagVisionIO() {},
                    new AprilTagVisionIO() {},
                    new AprilTagVisionIO() {},
                    new AprilTagVisionIO() {});
        case DEVBOT ->
            aprilTagVision =
                new AprilTagVision(
                    this::getAprilTagLayoutType,
                    new AprilTagVisionIO() {},
                    new AprilTagVisionIO() {});
        default -> aprilTagVision = new AprilTagVision(this::getAprilTagLayoutType);
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
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (backpackActuator == null) {
      backpackActuator = new BackpackActuator(new BackpackActuatorIO() {});
    }
    rollers = new Rollers(feeder, indexer, intake, backpack, rollersSensorsIO);
    superstructure = new Superstructure(arm, climber, backpackActuator);

    // Set up subsystems
    armCoast
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        armCoastOverride = true;
                      }
                    })
                .ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> armCoastOverride = false).ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(Commands.runOnce(() -> armCoastOverride = false).ignoringDisable(true));
    arm.setOverrides(armDisable, () -> armCoastOverride);
    climber.setCoastOverride(() -> armCoastOverride);
    backpackActuator.setCoastOverride(() -> armCoastOverride);
    robotState.setLookaheadDisable(lookaheadDisable);
    flywheels.setPrepareShootSupplier(
        () ->
            autoFlywheelSpinupDisable.negate().getAsBoolean()
                && DriverStation.isTeleopEnabled()
                && robotState
                        .getEstimatedPose()
                        .getTranslation()
                        .getDistance(
                            AllianceFlipUtil.apply(
                                FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
                    < Units.feetToMeters(25.0)
                && rollers.getGamepieceState() == GamepieceState.SHOOTER_STAGED
                && superstructure.getCurrentGoal() != Superstructure.Goal.PREPARE_CLIMB
                && superstructure.getCurrentGoal() != Superstructure.Goal.PREPARE_PREPARE_TRAP_CLIMB
                && superstructure.getCurrentGoal() != Superstructure.Goal.POST_PREPARE_TRAP_CLIMB
                && superstructure.getCurrentGoal() != Superstructure.Goal.CLIMB
                && superstructure.getCurrentGoal() != Superstructure.Goal.TRAP
                && superstructure.getCurrentGoal() != Superstructure.Goal.UNTRAP);

    // Configure autos and buttons
    configureAutos();
    configureButtonBindings();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }

    // Endgame alert triggers
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> Leds.getInstance().endgameAlert = true)
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9) // Rumble three times
                .beforeStarting(() -> Leds.getInstance().endgameAlert = true)
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));
  }

  private void configureAutos() {
    AutoBuilder autoBuilder =
        new AutoBuilder(drive, superstructure, flywheels, rollers, autoSelector::getResponses);

    // Add autos
    autoSelector.addRoutine(
        "Davis Spiky Auto",
        List.of(
            new AutoQuestion(
                "Starting location?",
                List.of(
                    AutoQuestionResponse.AMP,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.SOURCE)),
            new AutoQuestion(
                "How many spike notes?",
                List.of(AutoQuestionResponse.TWO, AutoQuestionResponse.THREE)),
            new AutoQuestion(
                "First center note?",
                List.of(
                    AutoQuestionResponse.AMP_WALL,
                    AutoQuestionResponse.AMP_MIDDLE,
                    AutoQuestionResponse.MIDDLE)),
            new AutoQuestion(
                "Second center note?",
                List.of(
                    AutoQuestionResponse.AMP_WALL,
                    AutoQuestionResponse.AMP_MIDDLE,
                    AutoQuestionResponse.MIDDLE))),
        autoBuilder.davisSpikyAuto());
    autoSelector.addRoutine(
        "Davis Speedy Auto",
        List.of(
            new AutoQuestion(
                "End behavior?",
                List.of(AutoQuestionResponse.SCORE_POOPED, AutoQuestionResponse.FOURTH_CENTER))),
        autoBuilder.davisSpeedyAuto());
    autoSelector.addRoutine("Davis Ethical Auto", autoBuilder.davisEthicalAuto());
    autoSelector.addRoutine(
        "Davis Unethical Auto",
        List.of(
            new AutoQuestion(
                "First center note?",
                List.of(AutoQuestionResponse.SOURCE_WALL, AutoQuestionResponse.SOURCE_MIDDLE))),
        autoBuilder.davisUnethicalAuto());

    // Set up feedforward characterization
    autoSelector.addRoutine(
        "Drive Static Characterization",
        new StaticCharacterization(
                drive, drive::runCharacterization, drive::getCharacterizationVelocity)
            .finallyDo(drive::endCharacterization));
    autoSelector.addRoutine(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
                drive, drive::runCharacterization, drive::getCharacterizationVelocity)
            .finallyDo(drive::endCharacterization));
    autoSelector.addRoutine(
        "Flywheels FF Characterization",
        new FeedForwardCharacterization(
            flywheels, flywheels::runCharacterization, flywheels::getCharacterizationVelocity));
    autoSelector.addRoutine(
        "Arm Static Characterization",
        new StaticCharacterization(
                superstructure,
                superstructure::runArmCharacterization,
                superstructure::getArmCharacterizationVelocity)
            .finallyDo(superstructure::endArmCharacterization));
    autoSelector.addRoutine(
        "Drive Wheel Radius Characterization",
        drive
            .orientModules(Drive.getCircleOrientations())
            .andThen(
                new WheelRadiusCharacterization(
                    drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE))
            .withName("Drive Wheel Radius Characterization"));
    autoSelector.addRoutine(
        "Diagnose Arm", superstructure.setGoalCommand(Superstructure.Goal.DIAGNOSTIC_ARM));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // ------------- Driver Controls -------------
    drive.setDefaultCommand(
        drive
            .run(
                () ->
                    drive.acceptTeleopInput(
                        -driver.getLeftY(),
                        -driver.getLeftX(),
                        -driver.getRightX(),
                        robotRelative.getAsBoolean()))
            .withName("Drive Teleop Input"));

    // ------------- Shooting Controls -------------
    // Aim and rev flywheels
    Supplier<Command> superstructureAimCommand =
        () ->
            Commands.either(
                Commands.either(
                    superstructure.setGoalCommand(Superstructure.Goal.PODIUM),
                    superstructure.setGoalCommand(Superstructure.Goal.SUBWOOFER),
                    () -> podiumShotMode),
                superstructure.setGoalCommand(Superstructure.Goal.AIM),
                shootPresets);
    Supplier<Command> driveAimCommand =
        () ->
            Commands.either(
                Commands.none(),
                Commands.startEnd(
                    () ->
                        drive.setHeadingGoal(() -> robotState.getAimingParameters().driveHeading()),
                    drive::clearHeadingGoal),
                shootAlignDisable);
    Trigger inWing =
        new Trigger(
            () ->
                AllianceFlipUtil.apply(robotState.getEstimatedPose().getX())
                    < FieldConstants.wingX);
    driver
        .a()
        .and(inWing)
        .whileTrue(
            driveAimCommand
                .get()
                .alongWith(superstructureAimCommand.get(), flywheels.shootCommand())
                .withName("Prepare Shot"));
    Translation2d superPoopTarget =
        FieldConstants.Speaker.centerSpeakerOpening
            .toTranslation2d()
            .interpolate(FieldConstants.ampCenter, 0.5);
    driver
        .a()
        .and(inWing.negate())
        .whileTrue(
            Commands.startEnd(
                    () ->
                        drive.setHeadingGoal(
                            () ->
                                AllianceFlipUtil.apply(superPoopTarget)
                                    .minus(robotState.getEstimatedPose().getTranslation())
                                    .getAngle()),
                    drive::clearHeadingGoal)
                .alongWith(
                    superstructure.setGoalCommand(Superstructure.Goal.SUPER_POOP),
                    flywheels.superPoopCommand())
                .withName("Super Poop"));
    Trigger readyToShoot =
        new Trigger(
            () -> drive.atHeadingGoal() && superstructure.atArmGoal() && flywheels.atGoal());
    driver
        .rightTrigger()
        .and(driver.a())
        .and(inWing)
        .and(readyToShoot)
        .onTrue(
            Commands.parallel(
                    Commands.waitSeconds(0.5), Commands.waitUntil(driver.rightTrigger().negate()))
                .deadlineWith(
                    rollers.setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER),
                    superstructureAimCommand.get(),
                    flywheels.shootCommand()));
    driver
        .rightTrigger()
        .and(driver.a())
        .and(inWing.negate())
        .and(readyToShoot)
        .onTrue(
            Commands.parallel(
                    Commands.waitSeconds(0.5), Commands.waitUntil(driver.rightTrigger().negate()))
                .deadlineWith(
                    rollers.setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER),
                    superstructure.setGoalCommand(Superstructure.Goal.SUPER_POOP),
                    flywheels.superPoopCommand()));
    driver.a().and(readyToShoot).whileTrue(controllerRumbleCommand());

    // Poop.
    driver
        .rightTrigger()
        .and(driver.a().negate())
        .and(driver.b().negate())
        .whileTrue(
            flywheels
                .poopCommand()
                .alongWith(
                    Commands.waitUntil(flywheels::atGoal)
                        .andThen(rollers.setGoalCommand(Rollers.Goal.FEED_TO_SHOOTER))));

    // ------------- Intake Controls -------------
    // Intake Floor
    driver
        .leftTrigger()
        .and(
            DriverStation
                ::isEnabled) // Must be enabled, allowing driver to hold button as soon as auto ends
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.INTAKE)
                .alongWith(
                    Commands.waitUntil(superstructure::atArmGoal)
                        .andThen(rollers.setGoalCommand(Rollers.Goal.FLOOR_INTAKE)))
                .withName("Floor Intake"));
    driver
        .leftTrigger()
        .and(() -> rollers.getGamepieceState() != GamepieceState.NONE)
        .onTrue(controllerRumbleCommand().withTimeout(0.5));

    // Eject Floor
    driver
        .leftBumper()
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.INTAKE)
                .alongWith(
                    Commands.waitUntil(superstructure::atArmGoal)
                        .andThen(rollers.setGoalCommand(Rollers.Goal.EJECT_TO_FLOOR)))
                .withName("Eject To Floor"));

    // Intake source
    driver
        .rightBumper()
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.STATION_INTAKE)
                .alongWith(
                    rollers.setGoalCommand(Rollers.Goal.STATION_INTAKE), flywheels.intakeCommand())
                .withName("Source Intake"));

    // ------------- Amp Scoring Controls -------------
    Container<Translation2d> ampAlignedDriverTranslation = new Container<>();
    Container<Translation2d> ampAlignedDriverFeedforward = new Container<>();
    Supplier<Pose2d> ampAlignedPose =
        () -> {
          Pose2d ampCenterRotated =
              AllianceFlipUtil.apply(
                  new Pose2d(FieldConstants.ampCenter, new Rotation2d(-Math.PI / 2.0)));
          var finalPose =
              ampCenterRotated
                  .transformBy(GeomUtil.toTransform2d(Units.inchesToMeters(38.0), 0))
                  .transformBy(FudgeFactors.amp.getTransform());
          double distance =
              robotState
                  .getEstimatedPose()
                  .getTranslation()
                  .minus(ampAlignedDriverTranslation.value)
                  .getDistance(finalPose.getTranslation());
          double offsetT = MathUtil.clamp((distance - 0.3) / 2.5, 0.0, 1.0);
          return finalPose.transformBy(GeomUtil.toTransform2d(offsetT * 1.75, 0.0));
        };
    Command ampAutoDrive =
        Commands.runOnce(
                () -> {
                  ampAlignedDriverTranslation.value = new Translation2d();
                  ampAlignedDriverFeedforward.value = new Translation2d();
                })
            .andThen(
                Commands.parallel(
                    drive.startEnd(
                        () ->
                            drive.setAutoAlignGoal(
                                () -> {
                                  Pose2d goal = ampAlignedPose.get();
                                  Logger.recordOutput("AmpAlign/UnadjustedGoal", goal);
                                  goal =
                                      new Pose2d(
                                          goal.getTranslation()
                                              .plus(ampAlignedDriverTranslation.value),
                                          goal.getRotation());
                                  Logger.recordOutput("AmpAlign/Goal", goal);
                                  return goal;
                                },
                                () -> ampAlignedDriverFeedforward.value,
                                false),
                        drive::clearAutoAlignGoal),
                    Commands.waitSeconds(0.5)
                        .andThen(
                            Commands.run(
                                () -> {
                                  final double maxVelocity = 1.5;
                                  ampAlignedDriverFeedforward.value =
                                      TeleopDriveController.calcLinearVelocity(
                                              -driver.getLeftY() * 0.5, -driver.getLeftX())
                                          .times(maxVelocity)
                                          .rotateBy(
                                              AllianceFlipUtil.shouldFlip()
                                                  ? new Rotation2d(Math.PI)
                                                  : new Rotation2d());
                                  ampAlignedDriverTranslation.value =
                                      ampAlignedDriverTranslation.value.plus(
                                          ampAlignedDriverFeedforward.value.times(
                                              Constants.loopPeriodSecs));
                                }))));

    driver
        .b()
        .whileTrue(
            Commands.either(
                    // Drive while heading is being controlled
                    drive
                        .run(
                            () ->
                                drive.acceptTeleopInput(
                                    -driver.getLeftY(),
                                    -driver.getLeftX(),
                                    0.0,
                                    robotRelative.getAsBoolean()))
                        .alongWith(
                            Commands.startEnd(
                                () -> drive.setHeadingGoal(() -> Rotation2d.fromDegrees(-90.0)),
                                drive::clearHeadingGoal)),

                    // Auto drive to amp
                    ampAutoDrive,
                    autoDriveDisable)
                .alongWith(
                    Commands.waitUntil(
                            () -> {
                              if (autoDriveDisable.getAsBoolean()) {
                                return true;
                              }
                              Pose2d poseError =
                                  robotState.getEstimatedPose().relativeTo(ampAlignedPose.get());
                              return poseError.getTranslation().getNorm() <= Units.feetToMeters(5.0)
                                  && Math.abs(poseError.getRotation().getDegrees()) <= 120;
                            })
                        .andThen(
                            superstructure.setGoalWithConstraintsCommand(
                                Superstructure.Goal.AMP, Arm.smoothProfileConstraints.get()))));
    driver
        .rightTrigger()
        .and(driver.b())
        .and(
            () ->
                superstructure.getDesiredGoal() == Superstructure.Goal.AMP
                    && superstructure.atArmGoal())
        .whileTrue(rollers.setGoalCommand(Rollers.Goal.AMP_SCORE).onlyWhile(driver.rightTrigger()));

    // ------------- Climbing Controls -------------
    Command trapSequence =
        ClimbingCommands.trapSequence(
            drive, superstructure, rollers, driver.povUp(), driver.povDown());
    Command simpleSequence =
        ClimbingCommands.simpleSequence(superstructure, driver.povUp(), driver.povDown());
    driver
        .start()
        .and(driver.back())
        .onTrue(
            Commands.runOnce(
                () -> {
                  simpleSequence.cancel();
                  trapSequence.cancel();
                }));
    driver.x().doublePress().onTrue(trapSequence);
    driver.y().doublePress().onTrue(simpleSequence);

    // ------------- Operator Controls -------------
    // Adjust shot compensation
    operator
        .povUp()
        .whileTrue(
            Commands.runOnce(() -> robotState.adjustShotCompensationDegrees(0.1))
                .andThen(Commands.waitSeconds(0.05))
                .ignoringDisable(true)
                .repeatedly());
    operator
        .povDown()
        .whileTrue(
            Commands.runOnce(() -> robotState.adjustShotCompensationDegrees(-0.1))
                .andThen(Commands.waitSeconds(0.05))
                .ignoringDisable(true)
                .repeatedly());

    // Switch arm preset
    operator
        .y()
        .onTrue(Commands.runOnce(() -> podiumShotMode = true)); //  set preset mode to podium
    operator
        .a()
        .onTrue(Commands.runOnce(() -> podiumShotMode = false)); // set preset mode to subwoofer

    // Request amp
    operator
        .x()
        .whileTrue(Commands.startEnd(() -> leds.requestAmp = true, () -> leds.requestAmp = false));

    // Shuffle gamepiece
    operator.b().whileTrue(rollers.shuffle());

    // Unjam (untaco)
    operator
        .rightTrigger()
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.AMP)
                .alongWith(
                    Commands.waitUntil(superstructure::atArmGoal)
                        .andThen(rollers.setGoalCommand(Rollers.Goal.UNJAM_UNTACO)))
                .withName("Unjam (Untaco)"));

    // Unjam (feeder)
    operator
        .rightBumper()
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.UNJAM_FEEDER)
                .alongWith(rollers.setGoalCommand(Rollers.Goal.UNJAM_FEEDER))
                .withName("Unjam (Feeder)"));

    // Reset heading
    operator
        .start()
        .and(operator.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState.resetPose(
                            new Pose2d(
                                robotState.getEstimatedPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));
  }

  /** Creates a controller rumble command with specified rumble and controllers */
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());
  }

  /** Updates dashboard data. */
  public void updateDashboardOutputs() {
    SmartDashboard.putString(
        "Shot Compensation Degrees",
        String.format("%.1f", robotState.getShotCompensationDegrees()));
    SmartDashboard.putBoolean("Podium Preset", podiumShotMode);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use (" + getAprilTagLayoutType().toString() + ").");
    }

    // Ridiculous auto alert
    ridiculousAutoAlert.set(
        autoSelector.getSelectedName().equals("Davis Spiky Auto")
            && autoSelector.getResponses().get(2) == autoSelector.getResponses().get(3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getCommand();
  }
}
