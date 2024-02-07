package frc.robot.subsystems.superstructure.Arm;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.ArmConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Arm/kP", controllerConstants.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Arm/Ki", controllerConstants.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Arm/kD", controllerConstants.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/Ks", controllerConstants.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/Kv", controllerConstants.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/Ka", controllerConstants.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/Kg", controllerConstants.ffkG());
  private static final LoggedTunableNumber armVelocity =
      new LoggedTunableNumber("Arm/Velocity", profileConstraints.cruiseVelocityRadPerSec());
  private static final LoggedTunableNumber armAcceleration =
      new LoggedTunableNumber("Arm/Acceleration", profileConstraints.accelerationRadPerSec2());
  private static final LoggedTunableNumber armTolerance =
      new LoggedTunableNumber("Arm/Tolerance", positionTolerance.getRadians());
  private static final LoggedTunableNumber armDesiredSetpoint =
      new LoggedTunableNumber("Arm/SetpointDegrees", 0.0);

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private BooleanSupplier disableSupplier = () -> false;
  private BooleanSupplier coastSupplier = () -> false;

  private final Mechanism2d armMechanism;
  private final MechanismRoot2d armRoot;
  private final MechanismLigament2d armMeasured;

  private final Timer homingTimer = new Timer();

  private boolean homed = false;
  private double setpoint = 0.0;

  //  private final MechanismLigament2d armSetpoint;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.armIO = io;
    io.setBrakeMode(true);

    // Create a mechanism
    armMechanism = new Mechanism2d(2, 3, new Color8Bit(Color.kAntiqueWhite));
    armRoot =
        armMechanism.getRoot(
            "Arm Joint",
            armOrigin2d.getX() + DriveConstants.driveConfig.trackwidthX() / 2.0,
            armOrigin2d.getY());
    armMeasured =
        new MechanismLigament2d(
            "Arm Measured",
            armLength,
            Units.radiansToDegrees(inputs.armAnglePositionRads),
            2.0,
            new Color8Bit(Color.kBlack));
    armRoot.append(armMeasured);
    //    armSetpoint =
    //        new MechanismLigament2d(
    //            "Arm Setpoint",
    //            armLength,
    //            inputs.armReferencePosition.getDegrees(),
    //            10.0,
    //            new Color8Bit(Color.kGreen));
    //    armRoot.append(armSetpoint);
  }

  @Override
  public void periodic() {
    // Process inputs
    armIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> armIO.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> armIO.setFF(kS.get(), kV.get(), kA.get(), kG.get()), kS, kV, kA, kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        constraints -> armIO.setProfileConstraints(constraints[0], constraints[1]),
        armVelocity,
        armAcceleration);

    // Home if not already homed
    if (!homed && DriverStation.isEnabled()) {
      armIO.setVoltage(-1.0);
      if (EqualsUtil.epsilonEquals(
          inputs.armVelocityRadsPerSec, 0.0, Units.degreesToRadians(1.0))) {
        homingTimer.start();
      } else {
        homingTimer.reset();
      }

      if (homingTimer.hasElapsed(0.5)) {
        armIO.setPosition(0);
        homed = true;
      }
    } else {
      armIO.setSetpoint(setpoint);
    }

    if (DriverStation.isDisabled()) {
      armIO.stop();
    }

    if (coastSupplier.getAsBoolean()) armIO.setBrakeMode(false);

    // Logs
    armMeasured.setAngle(Units.radiansToDegrees(inputs.armAnglePositionRads));
    //    armSetpoint.setAngle(inputs.armReferencePosition);
    Logger.recordOutput("Arm/Mechanism", armMechanism);
  }

  public void setVoltage(double volts) {
    armIO.setVoltage(volts);
  }

  public void setSetpoint(double setpointRads) {
    if (disableSupplier.getAsBoolean() || !homed) return;
  }

  public void setBrakeMode(boolean enabled) {
    armIO.setBrakeMode(enabled);
  }

  public void setOverrides(BooleanSupplier disableSupplier, BooleanSupplier coastSupplier) {
    this.disableSupplier = disableSupplier;
    this.coastSupplier = coastSupplier;
  }

  public void stop() {
    armIO.stop();
  }

  public Command getStaticCurrent() {
    Timer timer = new Timer();
    return run(() -> armIO.setCurrent(0.5 * timer.get()))
        .beforeStarting(timer::restart)
        .until(() -> Math.abs(inputs.armVelocityRadsPerSec) >= Units.degreesToRadians(10))
        .andThen(() -> Logger.recordOutput("Arm/staticCurrent", inputs.armTorqueCurrentAmps[0]))
        .andThen(armIO::stop);
  }

  @AutoLogOutput(key = "Arm/Homed")
  public boolean homed() {
    return homed;
  }

  @AutoLogOutput(key = "Arm/AtSetpoint")
  public boolean atSetpoint() {
    return Math.abs(inputs.armAnglePositionRads - setpoint) <= armTolerance.get();
  }
}
