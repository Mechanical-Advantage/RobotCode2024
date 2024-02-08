package org.littletonrobotics.frc2024.subsystems.superstructure.shooter;

import static org.littletonrobotics.frc2024.subsystems.superstructure.SuperstructureConstants.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static final LoggedTunableNumber feedVolts =
            new LoggedTunableNumber("Shooter/FeedVolts", 6.0);
    private static final LoggedTunableNumber leftkP =
            new LoggedTunableNumber("Shooter/leftkP", leftFlywheelConstants.kP());
    private static final LoggedTunableNumber leftkI =
            new LoggedTunableNumber("Shooter/leftkI", leftFlywheelConstants.kI());
    private static final LoggedTunableNumber leftkD =
            new LoggedTunableNumber("Shooter/leftkD", leftFlywheelConstants.kD());
    private static final LoggedTunableNumber leftkS =
            new LoggedTunableNumber("Shooter/leftkS", leftFlywheelConstants.kS());
    private static final LoggedTunableNumber leftkV =
            new LoggedTunableNumber("Shooter/leftkV", leftFlywheelConstants.kV());
    private static final LoggedTunableNumber leftkA =
            new LoggedTunableNumber("Shooter/leftkA", leftFlywheelConstants.kA());
    private static final LoggedTunableNumber rightkP =
            new LoggedTunableNumber("Shooter/rightkP", rightFlywheelConstants.kP());
    private static final LoggedTunableNumber rightkI =
            new LoggedTunableNumber("Shooter/rightkI", rightFlywheelConstants.kI());
    private static final LoggedTunableNumber rightkD =
            new LoggedTunableNumber("Shooter/rightkD", rightFlywheelConstants.kD());
    private static final LoggedTunableNumber rightkS =
            new LoggedTunableNumber("Shooter/rightkS", rightFlywheelConstants.kS());
    private static final LoggedTunableNumber rightkV =
            new LoggedTunableNumber("Shooter/rightkV", rightFlywheelConstants.kV());
    private static final LoggedTunableNumber rightkA =
            new LoggedTunableNumber("Shooter/rightkA", rightFlywheelConstants.kA());
    private static final LoggedTunableNumber shooterTolerance =
            new LoggedTunableNumber("Shooter/ToleranceRPM", shooterToleranceRPM);
    private static final LoggedTunableNumber intakeVolts =
            new LoggedTunableNumber("Shooter/IntakeVolts", -5.0);

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private boolean spinning = false;
    private boolean intaking = false;

    private boolean feeding = false;
    private double leftRPM = 0.0;
    private double rightRPM = 0.0;

    public Shooter(ShooterIO io) {
        System.out.println("[Init] Creating Shooter");
        this.io = io;
        this.io.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get());
        this.io.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get());
        this.io.setRightFF(rightkS.get(), rightkV.get(), rightkA.get());
        this.io.setRightPID(rightkP.get(), rightkI.get(), rightkD.get());
    }

    @Override
    public void periodic() {
        // check controllers
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get()),
                leftkP,
                leftkI,
                leftkD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get()),
                leftkS,
                leftkV,
                leftkA);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setRightPID(rightkP.get(), rightkI.get(), rightkD.get()),
                rightkP,
                rightkI,
                rightkD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setRightFF(rightkS.get(), rightkV.get(), rightkA.get()),
                rightkS,
                rightkV,
                rightkA);

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        } else {
            if (spinning) {
                io.setRPM(leftRPM, rightRPM);
                if (feeding) {
                    io.setFeederVoltage(feedVolts.get());
                }
            } else if (intaking) {
                io.setLeftVoltage(intakeVolts.get());
                io.setRightVoltage(intakeVolts.get());
            }
        }

        Logger.recordOutput("Shooter/LeftRPM", inputs.leftFlywheelVelocityRPM);
        Logger.recordOutput("Shooter/RightRPM", inputs.rightFlywheelVelocityRPM);
        Logger.recordOutput("Shooter/FeederRPM", inputs.feederVelocityRPM);
    }

    public void requestRPM(double leftRPM, double rightRPM) {
        intaking = false;
        spinning = true;
        this.leftRPM = leftRPM;
        this.rightRPM = rightRPM;
    }

    public void requestFeed() {
        intaking = false;
        feeding = true;
    }

    public void requestIntake() {
        spinning = false;
        intaking = true;
    }

    public void stop() {
        spinning = false;
        intaking = false;
    }

    public void runLeftCharacterizationVolts(double volts) {
        spinning = false;
        intaking = false;
        io.setLeftVoltage(volts);
    }

    public void runRightCharacterizationVolts(double volts) {
        spinning = false;
        intaking = false;
        io.setRightVoltage(volts);
    }

    public double getLeftCharacterizationVelocity() {
        return inputs.leftFlywheelVelocityRPM;
    }

    public double getRightCharacterizationVelocity() {
        return inputs.rightFlywheelVelocityRPM;
    }

    @AutoLogOutput(key = "Shooter/AtSetpoint")
    public boolean atSetpoint() {
        return Math.abs(inputs.leftFlywheelVelocityRPM - leftRPM) <= shooterTolerance.get()
                && Math.abs(inputs.rightFlywheelVelocityRPM - rightRPM) <= shooterTolerance.get()
                && spinning;
    }
}
