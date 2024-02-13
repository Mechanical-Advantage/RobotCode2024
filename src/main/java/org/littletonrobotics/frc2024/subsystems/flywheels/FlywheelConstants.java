package org.littletonrobotics.frc2024.subsystems.flywheels;

import org.littletonrobotics.frc2024.Constants;

public class FlywheelConstants {
    public static double reduction = (1.0 / 2.0);
    public static double shooterToleranceRPM = 100.0;
    public static int leftID = 5;
    public static int rightID = 4;

    public static Config config =
            switch (Constants.getRobot()) {
                case COMPBOT -> null;
                case DEVBOT -> new Config(5, 4, (1.0 / 2.0), 100.0);
                case SIMBOT -> new Config(0, 0, (1.0 / 2.0), 100.0);
            };

    public static Gains gains =
            switch (Constants.getRobot()) {
                case COMPBOT -> null;
                case DEVBOT -> new Gains(0.0006, 0.0, 0.05, 0.33329, 0.00083, 0.0);
                case SIMBOT -> new Gains(0.0, 0.0, 0.0, 0.09078, 0.00103, 0.0);
            };

    public record Config(int leftID, int rightID, double reduction, double toleranceRPM) {}
    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {
    }
}
