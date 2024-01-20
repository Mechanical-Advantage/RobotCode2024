package frc.robot.subsystems.shooter;

public class ShooterConstants {
  // encoder / flywheelReduction = flywheel
  public static double flywheelReduction = (1.0 / 2.0);

  public static int leftMotorId = 2;
  public static int rightMotorId = 1;
  public static int feederMotorId = 3;

  public static boolean leftMotorInverted = false;
  public static boolean rightMotorInverted = false;
  public static boolean feederMotorInverted = false;

  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.33329;
  public static double kV = 0.00083;
  public static double kA = 0.0;

  public static double shooterToleranceRPM = 50.0;
}
