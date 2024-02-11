package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public double yawPositionRad = 0;
    public double pitchPositionRad = 0;
    public double rollPositionRad = 0;
    public double yawVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double rollVelocityRadPerSec = 0.0;
  }

  default void updateInputs(GyroIOInputs inputs) {}

  default void setGyro(GyroIOInputs inputs, double degrees) {
    inputs.yawPositionRad = Math.toRadians(degrees);
  }

  default void resetGyro() {}
}
