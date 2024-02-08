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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import org.littletonrobotics.frc2024.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2024.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2024.subsystems.drive.*;
import org.littletonrobotics.frc2024.subsystems.superstructure.DevBotSuperstructure;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.Arm;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOKrakenFOC;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.intake.Intake;
import org.littletonrobotics.frc2024.subsystems.superstructure.intake.IntakeIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.intake.IntakeIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.shooter.Shooter;
import org.littletonrobotics.frc2024.subsystems.superstructure.shooter.ShooterIO;
import org.littletonrobotics.frc2024.subsystems.superstructure.shooter.ShooterIOSim;
import org.littletonrobotics.frc2024.subsystems.superstructure.shooter.ShooterIOSparkMax;
import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
import org.littletonrobotics.frc2024.util.trajectory.ChoreoTrajectoryReader;
import org.littletonrobotics.frc2024.util.trajectory.HolonomicTrajectory;
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
    private DevBotSuperstructure superstructure;

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
                //        intake = new Intake(new IntakeIOSparkMax());
                superstructure = new DevBotSuperstructure(arm, shooter);
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
                superstructure = new DevBotSuperstructure(arm, shooter);
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

        controller.rightBumper().onTrue(intake.intakeCommand()).onFalse(intake.stopCommand());

        controller
                .leftTrigger()
                .onTrue(Commands.runOnce(drive::setAutoAimGoal))
                .onFalse(Commands.runOnce(drive::clearAutoAimGoal));

        if (superstructure != null) {
            controller
                    .leftTrigger()
                    .onTrue(
                            Commands.runOnce(
                                    () -> superstructure.setDesiredState(DevBotSuperstructure.State.PREPARE_SHOOT)))
                    .onFalse(
                            Commands.runOnce(
                                    () -> superstructure.setDesiredState(DevBotSuperstructure.State.IDLE)));

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
                                            () -> superstructure.setDesiredState(DevBotSuperstructure.State.SHOOT))
                                    .andThen(Commands.waitSeconds(0.5))
                                    .andThen(
                                            Commands.runOnce(
                                                    () -> superstructure.setDesiredState(DevBotSuperstructure.State.IDLE))));

            controller
                    .leftBumper()
                    .onTrue(
                            Commands.runOnce(
                                    () -> superstructure.setDesiredState(DevBotSuperstructure.State.INTAKE)))
                    .onFalse(
                            Commands.runOnce(
                                    () -> superstructure.setDesiredState(DevBotSuperstructure.State.IDLE)));
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
