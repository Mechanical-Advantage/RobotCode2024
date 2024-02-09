// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.littletonrobotics.frc2024;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import org.littletonrobotics.frc2024.util.Alert;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int loopPeriodMs = 20;
    private static RobotType robotType = RobotType.SIMBOT;
    public static final boolean tuningMode = true;

    private static boolean invalidRobotAlertSent = false;

    public static RobotType getRobot() {
        if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
            new Alert("Invalid Robot Selected, using COMPBOT as default", Alert.AlertType.ERROR)
                    .set(true);
            invalidRobotAlertSent = true;
            robotType = RobotType.COMPBOT;
        }
        return robotType;
    }

    public static Mode getMode() {
        return switch (robotType) {
            case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

    public static final Map<RobotType, String> logFolders = Map.of(RobotType.DEVBOT, "/media/sda1/");

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum RobotType {
        SIMBOT,
        DEVBOT,
        COMPBOT
    }

    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    /** Checks whether the robot the correct robot is selected when deploying. */
    public static void main(String... args) {
        if (robotType == RobotType.SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + robotType.toString());
            System.exit(1);
        }
    }
}
