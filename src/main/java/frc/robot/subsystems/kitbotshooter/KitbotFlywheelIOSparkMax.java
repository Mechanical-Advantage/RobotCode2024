package frc.robot.subsystems.kitbotshooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Arrays;

public class KitbotFlywheelIOSparkMax implements KitbotFlywheelIO {
  // FIND THESE CONSTANTS
  private static final boolean masterInverted = false;
  private static final boolean followerInverted = false;
  private static final SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  private static final PIDController feedback = new PIDController(0.0, 0.0, 0.0);
  public static final double GEARING = (1.0 / 1.0);

  private final CANSparkMax leader, follower;
  private final RelativeEncoder masterEncoder, followerEncoder;

  public KitbotFlywheelIOSparkMax() {
    // FIND ID
    leader = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    follower = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

    leader.restoreFactoryDefaults();
    leader.setCANTimeout(250);
    leader.setInverted(masterInverted);
    leader.setSmartCurrentLimit(80);
    leader.enableVoltageCompensation(12.0);

    follower.restoreFactoryDefaults();
    follower.setCANTimeout(250);
    follower.setInverted(followerInverted);
    follower.setSmartCurrentLimit(80);
    follower.enableVoltageCompensation(12.0);

    masterEncoder = leader.getEncoder();
    masterEncoder.setPosition(0.0);
    masterEncoder.setMeasurementPeriod(Constants.loopPeriodMs);
    masterEncoder.setAverageDepth(2);

    followerEncoder = follower.getEncoder();
    followerEncoder.setPosition(0.0);
    followerEncoder.setMeasurementPeriod(Constants.loopPeriodMs);
    followerEncoder.setAverageDepth(2);

    follower.follow(leader, true);
    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(KitbotFlywheelIOInputs inputs) {
    inputs.flywheelPositionRads = getPosition();
    inputs.flywheelVelocityRadPerSec = getVelocity();
    inputs.flywheelAppliedVolts =
        new double[] {leader.getAppliedOutput(), follower.getAppliedOutput()};
    inputs.flywheelCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    double ffVolts = ffModel.calculate(velocityRadPerSec);
    double pidEffort =
        feedback.calculate(Arrays.stream(getVelocity()).sum() / 2.0, velocityRadPerSec);
    leader.setVoltage(ffVolts + pidEffort);
  }

  @Override
  public void runVolts(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    leader.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    follower.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public double getVelocityRPM() {
    double motorRPS = (masterEncoder.getVelocity() + followerEncoder.getVelocity()) / 2.0;
    return Units.radiansPerSecondToRotationsPerMinute(motorRPS) / GEARING;
  }

  private double[] getPosition() {
    return new double[] {
      Units.rotationsToRadians(masterEncoder.getPosition()) / GEARING,
      Units.rotationsToRadians(followerEncoder.getPosition()) / GEARING
    };
  }

  private double[] getVelocity() {
    return new double[] {
      Units.rotationsToRadians(masterEncoder.getVelocity()) / GEARING,
      Units.rotationsToRadians(followerEncoder.getVelocity()) / GEARING
    };
  }
}
