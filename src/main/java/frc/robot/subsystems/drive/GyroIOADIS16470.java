package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;

public class GyroIOADIS16470 implements GyroIO {
  private final ADIS16470_IMU gyro;

  public GyroIOADIS16470() {
    gyro =
        new ADIS16470_IMU(
            ADIS16470_IMU.IMUAxis.kZ,
            ADIS16470_IMU.IMUAxis.kX,
            ADIS16470_IMU.IMUAxis.kY,
            SPI.Port.kOnboardCS0,
            ADIS16470_IMU.CalibrationTime._4s);
    gyro.calibrate();
    gyro.setGyroAngleZ(0);
  }

  public double getYawDegrees() {
    return Math.toDegrees(gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ));
  }

  public void updateInputs(GyroIOInputs gyroInputs) {
    gyroInputs.connected = gyro.isConnected();

    gyroInputs.yawPositionRad = Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ));
    gyroInputs.pitchPositionRad = Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kX));
    gyroInputs.rollPositionRad = Units.degreesToRadians(gyro.getAngle(ADIS16470_IMU.IMUAxis.kY));

    gyroInputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyro.getRate(ADIS16470_IMU.IMUAxis.kX));
    gyroInputs.pitchVelocityRadPerSec =
        Units.degreesToRadians(gyro.getRate(ADIS16470_IMU.IMUAxis.kY));
    gyroInputs.rollVelocityRadPerSec =
        Units.degreesToRadians(gyro.getRate(ADIS16470_IMU.IMUAxis.kZ));
  }
}
