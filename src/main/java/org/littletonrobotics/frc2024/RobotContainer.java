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
import java.util.function.Function;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2024.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2024.commands.auto.AutoBuilder;
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
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOSim;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.GeomUtil;
import org.littletonrobotics.frc2024.util.NoteVisualizer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({GeomUtil.class})
public class RobotContainer {
  // Load robot state
  private final RobotState robotState = RobotState.getInstance();

  // Subsystems
  private Drive drive;
  private AprilTagVision aprilTagVision;
  private Flywheels flywheels;
  private Rollers rollers;
  private final Superstructure superstructure;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final LoggedDashboardBoolean shotTuningMode =
      new LoggedDashboardBoolean("Shot Tuning Mode", false);
  private final LoggedDashboardBoolean autoAmpScore =
      new LoggedDashboardBoolean("Allow Auto Amp Score", false);

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
          indexer = new Indexer(new IndexerIODevbot());
          intake = new Intake(new IntakeIO() {});
          backpack = new Backpack(new BackpackIOSparkFlex());
          rollers = new Rollers(feeder, indexer, intake, backpack, new RollersSensorsIOReal());

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
          rollers = new Rollers(feeder, indexer, intake, backpack, new RollersSensorsIO() {});

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
      rollers = new Rollers(feeder, indexer, intake, backpack, new RollersSensorsIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    superstructure = new Superstructure(arm);

    // Configure NoteVisualizer
    NoteVisualizer.setRobotPoseSupplier(() -> RobotState.getInstance().getEstimatedPose());
    NoteVisualizer.setArmAngleSupplier(arm::getCurrentArmAngle);

    // Configure autos and buttons
    configureAutos();
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    AutoBuilder autoBuilder = new AutoBuilder(drive, superstructure, flywheels, rollers);

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
    autoChooser.addOption("Arm FF Characterization", superstructure.runArmCharacterization());
    autoChooser.addOption("Diagnose Arm", superstructure.diagnoseArm());

    autoChooser.addOption("Davis Ethical Auto", autoBuilder.davisEthicalAuto());
    autoChooser.addOption("N5_S1_C234", autoBuilder.N5_S1_C234());
    autoChooser.addOption("N5_S0_C012", autoBuilder.N5_S0_C012());
    autoChooser.addOption("N5_C432_S2", autoBuilder.N5_C432_S2());
    autoChooser.addOption("N6_S12-C0123", autoBuilder.N6_S12_C0123());
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
                        -driverController.getLeftY(),
                        -driverController.getLeftX(),
                        -driverController.getRightX()))
            .withName("Drive Teleop Input"));

    // Set up toggles
    Trigger inShotTuningMode = new Trigger(shotTuningMode::get);
    Trigger allowingAutoAmpScore = new Trigger(autoAmpScore::get);
    operatorController
        .start()
        .onTrue(Commands.runOnce(() -> shotTuningMode.set(!inShotTuningMode.getAsBoolean())));
    operatorController
        .x()
        .onTrue(Commands.runOnce(() -> autoAmpScore.set(!allowingAutoAmpScore.getAsBoolean())));

    // Aim and Rev Flywheels
    driverController
        .a()
        .and(inShotTuningMode.negate())
        .whileTrue(
            Commands.startEnd(
                    () ->
                        drive.setHeadingGoal(
                            () -> RobotState.getInstance().getAimingParameters().driveHeading()),
                    drive::clearHeadingGoal)
                .alongWith(superstructure.aim(), flywheels.shootCommand())
                .withName("Prepare Shot"));
    // Tuning mode controls
    driverController
        .a()
        .and(inShotTuningMode)
        .whileTrue(
            Commands.parallel(
                superstructure.diagnoseArm(),
                flywheels.shootCommand(),
                Commands.startEnd(
                    () ->
                        drive.setHeadingGoal(
                            () -> RobotState.getInstance().getAimingParameters().driveHeading()),
                    drive::clearHeadingGoal)));
    // Shoot
    Trigger readyToShoot =
        new Trigger(() -> drive.atHeadingGoal() && superstructure.atGoal() && flywheels.atGoal())
            .and(inShotTuningMode.negate());
    readyToShoot
        .whileTrue(
            Commands.run(
                () -> driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
        .whileFalse(
            Commands.run(
                () -> driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));
    // Shoot at current arm and flywheel setpoint
    driverController
        .rightTrigger()
        .and(driverController.a())
        .onTrue(
            Commands.parallel(
                    Commands.waitSeconds(0.5),
                    Commands.waitUntil(driverController.rightTrigger().negate()))
                .deadlineWith(rollers.feedShooter())
                .finallyDo(
                    () -> {
                      Logger.recordOutput(
                          "RobotState/ShootingArmAngle", superstructure.getArmAngle().getRadians());
                      Logger.recordOutput(
                          "RobotState/ShootingEffectiveDistance",
                          RobotState.getInstance().getAimingParameters().effectiveDistance());
                    })
                .withName("Shoot"));
    // Intake Floor
    driverController
        .leftTrigger()
        .whileTrue(
            superstructure
                .intake()
                .alongWith(
                    Commands.waitUntil(superstructure::atGoal).andThen(rollers.floorIntake()))
                .withName("Floor Intake"));
    // Eject Floor
    driverController
        .leftBumper()
        .whileTrue(
            superstructure
                .intake()
                .alongWith(Commands.waitUntil(superstructure::atGoal).andThen(rollers.ejectFloor()))
                .withName("Eject To Floor"));

    // Amp scoring
    driverController
        .rightBumper()
        .and(allowingAutoAmpScore.negate())
        .whileTrue(
            superstructure
                .amp()
                .alongWith(
                    Commands.startEnd(
                        () -> drive.setHeadingGoal(() -> new Rotation2d(-Math.PI / 2.0)),
                        drive::clearHeadingGoal)));
    driverController
        .rightBumper()
        .and(allowingAutoAmpScore.negate())
        .and(driverController.rightTrigger())
        .whileTrue(Commands.waitUntil(superstructure::atGoal).andThen(rollers.ampScore()));

    // Auto amp scoring
    Pose2d goalAmpScorePose =
        new Pose2d(
            FieldConstants.ampCenter.getX(),
            FieldConstants.ampCenter.getY()
                - DriveConstants.driveConfig.bumperWidthX() / 2.0
                - Units.inchesToMeters(5.0),
            new Rotation2d(-Math.PI / 2.0));
    Supplier<Pose2d> ampScoringPoseSupp = () -> AllianceFlipUtil.apply(goalAmpScorePose);
    driverController
        .rightBumper()
        .and(allowingAutoAmpScore)
        .whileTrue(
            Commands.startEnd(
                    () -> drive.setAutoAlignGoal(ampScoringPoseSupp.get()),
                    drive::clearAutoAlignGoal)
                .alongWith(
                    Commands.waitUntil(
                            () -> {
                              Pose2d poseError =
                                  RobotState.getInstance()
                                      .getEstimatedPose()
                                      .relativeTo(ampScoringPoseSupp.get());
                              return poseError.getTranslation().getNorm() < Units.feetToMeters(6.0)
                                  && poseError.getY() >= -Units.inchesToMeters(5.0);
                            })
                        .andThen(superstructure.amp()),
                    Commands.waitUntil(
                            () -> superstructure.atGoal() && drive.isAutoAlignGoalCompleted())
                        .andThen(rollers.ampScore())));

    // Operator controls
    Function<Double, Command> changeCompensationDegrees =
        compensation ->
            Commands.runOnce(
                    () -> {
                      double currentCompensation =
                          RobotState.getInstance().getShotCompensationDegrees();
                      RobotState.getInstance()
                          .setShotCompensationDegrees(currentCompensation + compensation);
                    })
                .ignoringDisable(true);
    operatorController.povDown().onTrue(changeCompensationDegrees.apply(-0.1));
    operatorController.povUp().onTrue(changeCompensationDegrees.apply(0.1));

    driverController
        .y()
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
