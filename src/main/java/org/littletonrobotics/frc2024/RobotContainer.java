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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
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
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);
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
  private boolean trapScoreMode = true;
  private boolean armCoastOverride = false;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

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
        () -> {
          return DriverStation.isTeleopEnabled()
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
              && superstructure.getCurrentGoal() != Superstructure.Goal.CLIMB
              && superstructure.getCurrentGoal() != Superstructure.Goal.TRAP
              && superstructure.getCurrentGoal() != Superstructure.Goal.CANCEL_PREPARE_CLIMB
              && superstructure.getCurrentGoal() != Superstructure.Goal.CANCEL_CLIMB;
        });

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
            controllerRumbleCommand(true, true)
                .withTimeout(0.5)
                .beforeStarting(() -> Leds.getInstance().endgameAlert = true)
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand(true, true)
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9) // Rumble three times
                .beforeStarting(() -> Leds.getInstance().endgameAlert = true)
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));
  }

  private void configureAutos() {
    AutoBuilder autoBuilder = new AutoBuilder(drive, superstructure, flywheels, rollers);

    autoChooser.addDefaultOption(
        "Do Nothing",
        Commands.runOnce(
            () ->
                robotState.resetPose(
                    new Pose2d(
                        new Translation2d(),
                        AllianceFlipUtil.apply(Rotation2d.fromDegrees(180.0))))));
    autoChooser.addOption("Davis Ethical Auto", autoBuilder.davisEthicalAuto());
    autoChooser.addOption("Davis Alternative Auto", autoBuilder.davisAlternativeAuto());

    // Spike autos
    autoChooser.addOption("Spike: Source 4", autoBuilder.source4());
    autoChooser.addOption("Spike: Center 4", autoBuilder.center4());
    autoChooser.addOption("Spike: Amp 4", autoBuilder.amp4());
    autoChooser.addOption("Spike: Source 5", autoBuilder.source5());
    autoChooser.addOption("Spike: Center 5", autoBuilder.center5());
    autoChooser.addOption("Spike: Amp 5", autoBuilder.amp5());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive Static Characterization",
        new StaticCharacterization(
                drive, drive::runCharacterization, drive::getCharacterizationVelocity)
            .finallyDo(drive::endCharacterization));
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
    autoChooser.addOption(
        "Diagnose Arm", superstructure.setGoalCommand(Superstructure.Goal.DIAGNOSTIC_ARM));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // ------------- Driver Controls -------------
    // Drive command
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
    driver.a().and(readyToShoot).whileTrue(controllerRumbleCommand(true, true));
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
    driver
        .a()
        .and(readyToShoot)
        .whileTrue(controllerRumbleCommand(true, true));

    // Poop.
    driver
        .y()
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
        .onTrue(controllerRumbleCommand(true, true)
                .withTimeout(0.5));

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
    Supplier<Pose2d> ampAlignedPose =
        () -> {
          Pose2d ampCenterRotated =
              AllianceFlipUtil.apply(
                  new Pose2d(FieldConstants.ampCenter, new Rotation2d(-Math.PI / 2.0)));
          var finalPose =
              ampCenterRotated
                  .transformBy(GeomUtil.toTransform2d(Units.inchesToMeters(20.0), 0))
                  .transformBy(FudgeFactors.amp.getTransform());
          double distance =
              robotState
                  .getEstimatedPose()
                  .getTranslation()
                  .getDistance(finalPose.getTranslation());
          double offsetT = MathUtil.clamp((distance - 0.3) / 2.5, 0.0, 1.0);
          return finalPose.transformBy(GeomUtil.toTransform2d(offsetT * 1.75, 0.0));
        };
    driver
        .b()
        .whileTrue(
            Commands.either(
                    drive.run(
                        () ->
                            drive.acceptTeleopInput(
                                -driver.getLeftY(),
                                -driver.getLeftX(),
                                -driver.getRightX(),
                                robotRelative.getAsBoolean())),
                    drive
                        .startEnd(
                            () -> drive.setAutoAlignGoal(ampAlignedPose, false),
                            drive::clearAutoAlignGoal)
                        .until(drive::isAutoAlignGoalCompleted)
                        .andThen(
                            drive.run(
                                () ->
                                    drive.acceptTeleopInput(
                                        -driver.getLeftY() * 0.5,
                                        -driver.getLeftX() * 0.5,
                                        0.0,
                                        robotRelative.getAsBoolean()))),
                    autoDriveDisable)
                .alongWith(
                    Commands.waitUntil(
                            () -> {
                              if (autoDriveDisable.getAsBoolean()) return true;
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
    driver
        .x()
        .and(
            () ->
                superstructure.getCurrentGoal() != Superstructure.Goal.CANCEL_CLIMB
                    && superstructure.getCurrentGoal() != Superstructure.Goal.CANCEL_PREPARE_CLIMB)
        .toggleOnTrue(
            Commands.either(
                superstructure.setGoalCommand(Superstructure.Goal.PREPARE_PREPARE_TRAP_CLIMB),
                Commands.none(), // No function yet
                () -> trapScoreMode));

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

    // Adjust arm preset
    operator.x().onTrue(Commands.runOnce(() -> podiumShotMode = !podiumShotMode));

    // Climber controls
    operator.rightStick().onTrue(Commands.runOnce(() -> trapScoreMode = !trapScoreMode));
    operator
        .leftBumper()
        .doublePress()
        .and(() -> trapScoreMode)
        .toggleOnTrue(
            ClimbingCommands.climbNTrapSequence(
                    drive,
                    superstructure,
                    rollers,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX(),
                    operator.rightBumper().doublePress(),
                    operator.start().doublePress().or(operator.back().doublePress()),
                    autoDriveDisable)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    operator
        .leftBumper()
        .doublePress()
        .and(() -> !trapScoreMode)
        .toggleOnTrue(
            ClimbingCommands.simpleClimbSequence(
                    superstructure, operator.rightBumper().doublePress())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    operator.leftStick().onTrue(superstructure.setGoalCommand(Superstructure.Goal.RESET));

    // Request amp
    operator
        .b()
        .whileTrue(Commands.startEnd(() -> leds.requestAmp = true, () -> leds.requestAmp = false));

    // Shuffle gamepiece
    operator.a().whileTrue(rollers.shuffle());

    // Start flywheels
    operator.rightTrigger().and(driver.a().negate()).whileTrue(flywheels.shootCommand());

    // Unjam folded to shooter
    operator
        .leftTrigger()
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.AMP)
                .alongWith(
                    Commands.waitUntil(superstructure::atArmGoal)
                        .andThen(rollers.setGoalCommand(Rollers.Goal.UNTACO)))
                .withName("Untaco"));

    // Unjam intake
    operator
        .y()
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.UNJAM_INTAKE)
                .alongWith(rollers.setGoalCommand(Rollers.Goal.EJECT_FROM_FEEDER))
                .withName("Unjam From Feeder"));

    operator
        .povLeft()
        .or(operator.povRight())
        .whileTrue(
            superstructure
                .setGoalCommand(Superstructure.Goal.BACKPACK_OUT_UNJAM)
                .withName("Backpack Out Unjam"));

    // Reset heading
    driver
        .start()
        .and(driver.back())
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
  private Command controllerRumbleCommand(boolean useRightRumble, boolean rumbleOperator) {
    return Commands.startEnd(
        () -> {
          driver
              .getHID()
              .setRumble(useRightRumble ? RumbleType.kBothRumble : RumbleType.kLeftRumble, 1.0);
          if (rumbleOperator) {
            operator
                .getHID()
                .setRumble(useRightRumble ? RumbleType.kBothRumble : RumbleType.kLeftRumble, 1.0);
          }
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
    SmartDashboard.putBoolean("Trap Score Mode", trapScoreMode);
  }

  /** Updates the AprilTag alert. */
  public void updateAprilTagAlert() {
    boolean active = getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(active);
    if (active) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use (" + getAprilTagLayoutType().toString() + ").");
    }
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
