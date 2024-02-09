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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.frc2024.util.EqualsUtil;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static final LoggedTunableNumber kP =
            new LoggedTunableNumber("Arm/kP", controllerConstants.kP());
    private static final LoggedTunableNumber kI =
            new LoggedTunableNumber("Arm/kI", controllerConstants.kI());
    private static final LoggedTunableNumber kD =
            new LoggedTunableNumber("Arm/kD", controllerConstants.kD());
    private static final LoggedTunableNumber kS =
            new LoggedTunableNumber("Arm/kS", controllerConstants.ffkS());
    private static final LoggedTunableNumber kV =
            new LoggedTunableNumber("Arm/kV", controllerConstants.ffkV());
    private static final LoggedTunableNumber kA =
            new LoggedTunableNumber("Arm/kA", controllerConstants.ffkA());
    private static final LoggedTunableNumber kG =
            new LoggedTunableNumber("Arm/kG", controllerConstants.ffkG());
    private static final LoggedTunableNumber armVelocity =
            new LoggedTunableNumber("Arm/Velocity", profileConstraints.cruiseVelocityRadPerSec());
    private static final LoggedTunableNumber armAcceleration =
            new LoggedTunableNumber("Arm/Acceleration", profileConstraints.accelerationRadPerSec2());
    private static final LoggedTunableNumber armTolerance =
            new LoggedTunableNumber("Arm/Tolerance", positionTolerance.getRadians());
    private static final LoggedTunableNumber armLowerLimit =
            new LoggedTunableNumber("Arm/LowerLimitDegrees", 7.0);
    private static final LoggedTunableNumber armUpperLimit =
            new LoggedTunableNumber("Arm/UpperLimitDegrees", 90.0);

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private BooleanSupplier disableSupplier = () -> false;
    private BooleanSupplier coastSupplier = () -> false;

    private final Mechanism2d armMechanism;
    private final MechanismRoot2d armRoot;
    private final MechanismLigament2d armMeasured;

    private final Timer homingTimer = new Timer();

    private boolean homed = false;
    private double setpoint = armLowerLimit.get();

    //  private final MechanismLigament2d armSetpoint;

    public Arm(ArmIO io) {
        System.out.println("[Init] Creating Arm");
        this.armIO = io;
        io.setBrakeMode(true);

        // Create a mechanism
        armMechanism = new Mechanism2d(2, 3, new Color8Bit(Color.kAntiqueWhite));
        armRoot = armMechanism.getRoot("Arm Joint", armOrigin2d.getX(), armOrigin2d.getY());
        armMeasured =
                new MechanismLigament2d(
                        "Arm Measured",
                        armLength,
                        Units.radiansToDegrees(inputs.armAnglePositionRads),
                        2.0,
                        new Color8Bit(Color.kBlack));
        armRoot.append(armMeasured);
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
                armIO.setPosition(Units.degreesToRadians(armLowerLimit.get()));
                homed = true;
            }
        } else {
            setpoint =
                    MathUtil.clamp(
                            setpoint,
                            Units.degreesToRadians(armLowerLimit.get()),
                            Units.degreesToRadians(armUpperLimit.get()));
            armIO.setSetpoint(setpoint);
        }

        if (DriverStation.isDisabled()) {
            armIO.stop();
        }

        if (coastSupplier.getAsBoolean()) armIO.setBrakeMode(false);

        // Logs
        armMeasured.setAngle(Units.radiansToDegrees(inputs.armAnglePositionRads));
        Logger.recordOutput("Arm/Mechanism", armMechanism);
    }

    public void setVoltage(double volts) {
        armIO.setVoltage(volts);
    }

    public void setSetpoint(double setpointRads) {
        if (disableSupplier.getAsBoolean() || !homed) return;
        setpoint = setpointRads;
    }

    public void stop() {
        armIO.stop();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.armAnglePositionRads);
    }

    @AutoLogOutput(key = "Arm/SetpointAngle")
    public Rotation2d getSetpoint() {
        return Rotation2d.fromRadians(setpoint);
    }

    @AutoLogOutput(key = "Arm/Homed")
    public boolean homed() {
        return homed;
    }

    @AutoLogOutput(key = "Arm/AtSetpoint")
    public boolean atSetpoint() {
        return Math.abs(inputs.armAnglePositionRads - setpoint) <= armTolerance.get();
    }

    public Command getStaticCurrent() {
        Timer timer = new Timer();
        return run(() -> armIO.setCurrent(0.5 * timer.get()))
                .beforeStarting(timer::restart)
                .until(() -> Math.abs(inputs.armVelocityRadsPerSec) >= Units.degreesToRadians(10))
                .andThen(() -> Logger.recordOutput("Arm/staticCurrent", inputs.armTorqueCurrentAmps[0]))
                .andThen(armIO::stop);
    }
}
