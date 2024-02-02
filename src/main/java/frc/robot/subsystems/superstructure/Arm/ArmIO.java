package frc.robot.subsystems.superstructure.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

   @AutoLog
   class ArmIOInputs {

      public double armVelocityRadsPerSec = 0.0;
      public double armAppliedVolts = 0.0;
      public double[] armCurrentAmps = new double[] {};
      public double [] armTempCelcius = new double[] {};
      public Rotation2d armInternalPosition = new Rotation2d();
      public Rotation2d armAnglePosition = new Rotation2d();

   }
   default void updateInputs(ArmIOInputs inputs) {}
   default void setSetpoint(Rotation2d setpoint) {}
   default void setBrakeMode(boolean enabled) {}
   default void stop() {}



}
