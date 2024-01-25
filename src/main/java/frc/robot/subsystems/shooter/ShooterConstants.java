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

  public static double leftkP = 0.0;
  public static double leftkI = 0.0;
  public static double leftkD = 0.0;
  public static double leftkS = 0.33329;
  public static double leftkV = 0.00083;
  public static double leftkA = 0.0;

  public static double rightkP = 0.0;
  public static double rightkI = 0.0;
  public static double rightkD = 0.0;
  public static double rightkS = 0.33329;
  public static double rightkV = 0.00083;
  public static double rightkA = 0.0;

  public static double shooterToleranceRPM = 50.0;
}
