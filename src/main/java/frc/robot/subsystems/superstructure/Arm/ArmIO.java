package frc.robot.subsystems.superstructure.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  class ArmIOInputs {
    public Rotation2d armAnglePosition = new Rotation2d();
    public Rotation2d armReferencePosition = new Rotation2d();
    public double armVelocityRadsPerSec = 0.0;
    public boolean homed = false;
    public boolean atSetpoint = false;
    public double[] armAppliedVolts = new double[] {};
    public double[] armCurrentAmps = new double[] {};
    public double[] armTorqueCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  default void setSetpoint(double setpointRads) {}

  /** Run motors at voltage */
  default void setVoltage(double volts) {}

  /** Set brake mode enabled */
  default void setBrakeMode(boolean enabled) {}

  /** Set FF values */
  default void setFF(double s, double v, double a, double g) {}

  /** Set PID values */
  default void setPID(double p, double i, double d) {}

  /** Set MotionMagic constraints */
  default void setProfileConstraints(
      double cruiseVelocityRadsPerSec, double accelerationRadsPerSec2) {}

  /**
   * Stops motors
   *
   * @params none!
   */
  default void stop() {}

  /**
   * Set current position to home
   *
   * @params none!
   */
  default void setHome() {}
}
