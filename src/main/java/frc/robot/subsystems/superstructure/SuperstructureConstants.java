package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureConstants {

  public static class ShooterConstants {
    // encoder / flywheelReduction = flywheel
    public static double flywheelReduction = (1.0 / 2.0);
    public static double shooterToleranceRPM = 50.0;

    public static FlywheelConstants leftFlywheelConstants =
        switch (Constants.getRobot()) {
          default -> new FlywheelConstants(2, false, 0.0, 0.0, 0.0, 0.33329, 0.00083, 0.0);
        };

    public static FlywheelConstants rightFlywheelConstants =
        switch (Constants.getRobot()) {
          default -> new FlywheelConstants(1, false, 0.0, 0.0, 0.0, 0.33329, 0.00083, 0.0);
        };

    public static FeederConstants feederConstants =
        switch (Constants.getRobot()) {
          default -> new FeederConstants(3, false);
        };

    public record FlywheelConstants(
        int id,
        boolean inverted,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA) {}

    public record FeederConstants(int id, boolean inverted) {}
  }

  public static class IntakeConstants {
    public static double reduction = (1.0 / 1.0);
    public static int id =
        switch (Constants.getRobot()) {
          default -> 4;
        };
    public static boolean inverted =
        switch (Constants.getRobot()) {
          default -> false;
        };
  }
}
