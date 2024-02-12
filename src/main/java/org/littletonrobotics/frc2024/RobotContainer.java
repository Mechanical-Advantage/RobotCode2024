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
import org.littletonrobotics.frc2024.subsystems.rollers.Rollers;
import org.littletonrobotics.frc2024.subsystems.rollers.RollersSensorsIO;
import org.littletonrobotics.frc2024.subsystems.rollers.RollersSensorsIOReal;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.Feeder;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIO;
import org.littletonrobotics.frc2024.subsystems.rollers.feeder.FeederIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.Indexer;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIO;
import org.littletonrobotics.frc2024.subsystems.rollers.indexer.IndexerIOSparkFlex;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.Intake;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIO;
import org.littletonrobotics.frc2024.subsystems.rollers.intake.IntakeIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.flywheels.Flywheels;
import org.littletonrobotics.frc2024.subsystems.superstructure.flywheels.FlywheelsIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.flywheels.FlywheelsIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.flywheels.FlywheelsIOSparkFlex;
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

  private Feeder feeder;
  private Indexer indexer;
  private Intake intake;
  private Rollers rollers;

  private Flywheels flywheels;
  private Arm arm;
  private Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(false),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[0]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[1]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[2]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[3]));

          feeder = new Feeder(new FeederIOKrakenFOC());
          indexer = new Indexer(new IndexerIOSparkFlex());
          intake = new Intake(new IntakeIOKrakenFOC());
          rollers = new Rollers(feeder, indexer, intake, new RollersSensorsIOReal());

          arm = new Arm(new ArmIOKrakenFOC());
          flywheels = new Flywheels(new FlywheelsIOSparkFlex());
          superstructure = new Superstructure(arm, flywheels);

          aprilTagVision =
              new AprilTagVision(
                  new AprilTagVisionIONorthstar(
                      AprilTagVisionConstants.instanceNames[0],
                      AprilTagVisionConstants.cameraIds[0]),
                  new AprilTagVisionIONorthstar(
                      AprilTagVisionConstants.instanceNames[1],
                      AprilTagVisionConstants.cameraIds[1]));
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
          flywheels = new Flywheels(new FlywheelsIOSim());
          // intake = new Intake(new IntakeIOSim());
          // feeder = new Feeder(new FeederIOSim());
          superstructure = new Superstructure(arm, flywheels);
        }
        case COMPBOT -> {
          // No impl yet
        }
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

    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }

    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }

    if (feeder == null) {
      feeder = new Feeder(new FeederIO() {});
    }

    if (indexer == null) {
      indexer = new Indexer(new IndexerIO() {});
    }

    if (rollers == null) {
      rollers = new Rollers(feeder, indexer, intake, new RollersSensorsIO() {});
    }

    if (superstructure == null) {
      superstructure = new Superstructure(arm, flywheels);
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
                drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity)
            .finallyDo(drive::endCharacterization));
    autoChooser.addOption(
        "Left Flywheels FF Characterization",
        new FeedForwardCharacterization(
            flywheels,
            flywheels::runLeftCharacterizationVolts,
            flywheels::getLeftCharacterizationVelocity));
    autoChooser.addOption(
        "Right Flywheels FF Characterization",
        new FeedForwardCharacterization(
            flywheels,
            flywheels::runRightCharacterizationVolts,
            flywheels::getRightCharacterizationVelocity));
    autoChooser.addOption("Arm get static current", arm.getStaticCurrent());

    AutoCommands autoCommands = new AutoCommands(drive, superstructure);

    autoChooser.addOption("Drive Straight", autoCommands.driveStraight());

    // Testing autos paths
    // Function<File, Optional<Command>> trajectoryCommandFactory =
    //         trajectoryFile -> {
    //             Optional<HolonomicTrajectory> trajectory =
    //                     ChoreoTrajectoryDeserializer.deserialize(trajectoryFile);
    //             return trajectory.map(
    //                     traj ->
    //                             Commands.runOnce(
    //                                             () ->
    // robotState.resetPose(AllianceFlipUtil.apply(traj.getStartPose())))
    //                                     .andThen(Commands.runOnce(() ->
    // drive.setTrajectoryGoal(traj)))
    //                                     .until(drive::isTrajectoryGoalCompleted));
    //         };
    // final File rootTrajectoryDir = new File(Filesystem.getDeployDirectory(), "choreo");
    // for (File trajectoryFile : Objects.requireNonNull(rootTrajectoryDir.listFiles())) {
    //     trajectoryCommandFactory
    //             .apply(trajectoryFile)
    //             .ifPresent(command -> autoChooser.addOption(trajectoryFile.getName(), command));
    // }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        drive.run(
            () ->
                drive.setTeleopDriveGoal(
                    -controller.getLeftY(), -controller.getLeftX(), -controller.getRightX())));

    controller
        .a()
        .onTrue(Commands.runOnce(drive::setAutoAimGoal))
        .onFalse(Commands.runOnce(drive::clearAutoAimGoal));

    if (superstructure != null) {
      controller
          .a()
          .onTrue(
              Commands.runOnce(
                  () -> superstructure.setGoalState(Superstructure.SystemState.PREPARE_SHOOT)))
          .onFalse(
              Commands.runOnce(() -> superstructure.setGoalState(Superstructure.SystemState.IDLE)));

      Trigger readyToShootTrigger =
          new Trigger(() -> drive.isAutoAimGoalCompleted() && superstructure.atShootingSetpoint());
      readyToShootTrigger
          .whileTrue(
              Commands.run(
                  () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
          .whileFalse(
              Commands.run(
                  () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));
      controller
          .rightTrigger()
          .and(readyToShootTrigger)
          .onTrue(
              Commands.runOnce(
                      () -> {
                        superstructure.setGoalState(Superstructure.SystemState.SHOOT);
                        rollers.setGoal(Rollers.Goal.FEED_SHOOTER);
                      },
                      superstructure,
                      rollers)
                  .andThen(Commands.waitSeconds(1.0))
                  .andThen(
                      Commands.runOnce(
                          () -> {
                            superstructure.setGoalState(Superstructure.SystemState.IDLE);
                            rollers.setGoal(Rollers.Goal.IDLE);
                          })));

      controller
          .leftTrigger()
          .whileTrue(
              Commands.runOnce(
                      () -> superstructure.setGoalState(Superstructure.SystemState.INTAKE),
                      superstructure)
                  .andThen(
                      Commands.waitSeconds(0.25),
                      Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.FLOOR_INTAKE), rollers),
                      Commands.idle())
                  .finallyDo(
                      () -> {
                        rollers.setGoal(Rollers.Goal.IDLE);
                        superstructure.setGoalState(Superstructure.SystemState.IDLE);
                      }));

      controller
          .leftBumper()
          .whileTrue(
              Commands.runOnce(
                      () -> superstructure.setGoalState(Superstructure.SystemState.INTAKE),
                      superstructure)
                  .andThen(
                      Commands.waitSeconds(0.25),
                      Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECT_TO_FLOOR), rollers),
                      Commands.idle())
                  .finallyDo(
                      () -> {
                        rollers.setGoal(Rollers.Goal.IDLE);
                        superstructure.setGoalState(Superstructure.SystemState.IDLE);
                      }));

      //      controller
      //          .a()
      //          .onTrue(
      //              Commands.runOnce(
      //                  () -> {
      //                    superstructure.setGoalState(Superstructure.SystemState.REVERSE_INTAKE);
      //                    rollers.setGoal(Rollers.Goal.STATION_INTAKE);
      //                  }))
      //          .onFalse(
      //              Commands.runOnce(
      //                  () -> {
      //                    superstructure.setGoalState(Superstructure.SystemState.IDLE);
      //                    rollers.setGoal(Rollers.Goal.IDLE);
      //                  }));
    }

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
                                new Rotation2d())))));

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
