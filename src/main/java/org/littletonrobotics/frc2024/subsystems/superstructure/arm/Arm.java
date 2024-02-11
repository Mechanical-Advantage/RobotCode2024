package org.littletonrobotics.frc2024.subsystems.superstructure.arm;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
import lombok.Setter;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", gains.ffkS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", gains.ffkV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", gains.ffkA());
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", gains.ffkG());
  private static final LoggedTunableNumber armVelocity =
      new LoggedTunableNumber("Arm/Velocity", profileConstraints.cruiseVelocityRadPerSec());
  private static final LoggedTunableNumber armAcceleration =
      new LoggedTunableNumber("Arm/Acceleration", profileConstraints.accelerationRadPerSec2());
  private static final LoggedTunableNumber armToleranceDegreees =
      new LoggedTunableNumber("Arm/ToleranceDegrees", positionTolerance.getDegrees());
  private static final LoggedTunableNumber armLowerLimit =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", 15.0);
  private static final LoggedTunableNumber armUpperLimit =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", 90.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Mechanism2d armMechanism;
  private final MechanismRoot2d armRoot;
  private final MechanismLigament2d armMeasured;

  private boolean homed = false;
  @Setter private Rotation2d setpoint = null;

  //  private final MechanismLigament2d armSetpoint;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
    io.setBrakeMode(true);

    // Create a mechanism
    armMechanism = new Mechanism2d(2, 3, new Color8Bit(Color.kAntiqueWhite));
    armRoot = armMechanism.getRoot("Arm Joint", armOrigin.getX(), armOrigin.getY());
    armMeasured =
        new MechanismLigament2d(
            "Arm Measured",
            armLength,
            Units.radiansToDegrees(inputs.armPositionRads),
            2.0,
            new Color8Bit(Color.kBlack));
    armRoot.append(armMeasured);
  }

  @Override
  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setFF(kS.get(), kV.get(), kA.get(), kG.get()), kS, kV, kA, kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        constraints -> io.setProfileConstraints(constraints[0], constraints[1]),
        armVelocity,
        armAcceleration);

    if (setpoint != null) {
      io.setSetpoint(
          MathUtil.clamp(
              setpoint.getRadians(),
              Units.degreesToRadians(armLowerLimit.get()),
              Units.degreesToRadians(armUpperLimit.get())));
    }

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    // Logs
    armMeasured.setAngle(Units.radiansToDegrees(inputs.armPositionRads));
    Logger.recordOutput("Arm/Mechanism", armMechanism);
  }

  public void runVolts(double volts) {
    setpoint = null;
    io.runVolts(volts);
  }

  public void stop() {
    io.stop();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.armPositionRads);
  }

  @AutoLogOutput(key = "Arm/SetpointAngle")
  public Rotation2d getSetpoint() {
    return setpoint != null ? setpoint : new Rotation2d();
  }

  @AutoLogOutput(key = "Arm/Homed")
  public boolean homed() {
    return homed;
  }

  @AutoLogOutput(key = "Arm/AtSetpoint")
  public boolean atSetpoint() {
    return setpoint != null
        && Math.abs(Rotation2d.fromRadians(inputs.armPositionRads).minus(setpoint).getDegrees())
            <= armToleranceDegreees.get();
  }

  public Command getStaticCurrent() {
    Timer timer = new Timer();
    return run(() -> io.runCurrent(0.5 * timer.get()))
        .beforeStarting(timer::restart)
        .until(() -> Math.abs(inputs.armVelocityRadsPerSec) >= Units.degreesToRadians(10))
        .andThen(() -> Logger.recordOutput("Arm/staticCurrent", inputs.armTorqueCurrentAmps[0]))
        .andThen(io::stop);
  }
}
