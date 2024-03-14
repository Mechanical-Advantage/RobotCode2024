// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024;

import static org.littletonrobotics.frc2024.util.Alert.AlertType;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.frc2024.Constants.Mode;
import org.littletonrobotics.frc2024.subsystems.leds.Leds;
import org.littletonrobotics.frc2024.util.Alert;
import org.littletonrobotics.frc2024.util.BatteryTracker;
import org.littletonrobotics.frc2024.util.NoteVisualizer;
import org.littletonrobotics.frc2024.util.VirtualSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String batteryNameFile = "/home/lvuser/battery-name.txt";
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double canivoreErrorTimeThreshold = 0.5;
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;

  private Command autoCommand;
  private RobotContainer robotContainer;
  private double autoStart;
  private boolean autoMessagePrinted;
  private boolean batteryNameWritten = false;
  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private double teleStart;
  private static double teleElapsedTime = 0.0;

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert canivoreErrorAlert =
      new Alert("CANivore error detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", AlertType.WARNING);

  public static Trigger createTeleopTimeTrigger(double teleElapsedTime) {
    return new Trigger(
        () ->
            DriverStation.isFMSAttached()
                && DriverStation.isTeleopEnabled()
                && Robot.teleElapsedTime > teleElapsedTime);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Leds.getInstance();

    // Record metadata
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    Logger.recordMetadata("BatteryName", "BAT-" + BatteryTracker.scanBattery(1.5));
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new RLOGServer());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new RLOGServer());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), 0.01));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    // Check for battery alert
    if (Constants.getMode() == Mode.REAL
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)) {
      File file = new File(batteryNameFile);
      if (file.exists()) {
        // Read previous battery name
        String previousBatteryName = "";
        try {
          previousBatteryName =
              new String(Files.readAllBytes(Paths.get(batteryNameFile)), StandardCharsets.UTF_8);
        } catch (IOException e) {
          e.printStackTrace();
        }
        if (previousBatteryName.equals(BatteryTracker.getName())) {
          // Same battery, set alert
          sameBatteryAlert.set(true);
          Leds.getInstance().sameBattery = true;
        } else {
          // New battery, delete file
          file.delete();
        }
      }
    }

    RobotController.setBrownoutVoltage(6.0);
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();

    // Print auto duration
    if (autoCommand != null) {
      if (!autoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
        Leds.getInstance().autoFinished = true;
        Leds.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
      }
    }

    // Log aiming parameters
    var aimingParameters = RobotState.getInstance().getAimingParameters();
    Logger.recordOutput(
        "RobotState/AimingParameters/DriveHeading", aimingParameters.driveHeading());
    Logger.recordOutput("RobotState/AimingParameters/ArmAngle", aimingParameters.armAngle());
    Logger.recordOutput(
        "RobotState/AimingParameters/EffectiveDistance", aimingParameters.effectiveDistance());
    Logger.recordOutput(
        "RobotState/AimingParameters/DriveFeedVelocity", aimingParameters.driveFeedVelocity());

    // Robot container periodic methods
    robotContainer.checkControllers();
    robotContainer.updateDashboardOutputs();
    robotContainer.updateAprilTagAlert();

    // Update NoteVisualizer
    NoteVisualizer.showHeldNotes();

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Log CANivore status
    if (Constants.getMode() == Mode.REAL) {
      var canivoreStatus = CANBus.getStatus("canivore");
      Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.Status.getName());
      Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.BusUtilization);
      Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.BusOffCount);
      Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount);
      Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC);
      Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC);
      if (!canivoreStatus.Status.isOK()
          || canStatus.transmitErrorCount > 0
          || canStatus.receiveErrorCount > 0) {
        canivoreErrorTimer.restart();
      }
      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold)
              && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
    }

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
      Leds.getInstance().lowBatteryAlert = true;
    }

    // Write battery name if connected to field
    if (Constants.getMode() == Mode.REAL
        && !batteryNameWritten
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)
        && DriverStation.isFMSAttached()) {
      batteryNameWritten = true;
      try {
        FileWriter fileWriter = new FileWriter(batteryNameFile);
        fileWriter.write(BatteryTracker.getName());
        fileWriter.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    Threads.setCurrentThreadPriority(true, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }

    NoteVisualizer.resetAutoNotes();
    NoteVisualizer.setHasNote(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    NoteVisualizer.showAutoNotes();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    NoteVisualizer.clearAutoNotes();
    NoteVisualizer.showAutoNotes();

    teleStart = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    teleElapsedTime = Timer.getFPGATimestamp() - teleStart;
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
