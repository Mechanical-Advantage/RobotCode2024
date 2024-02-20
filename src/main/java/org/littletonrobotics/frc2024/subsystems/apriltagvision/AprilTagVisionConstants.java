// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2024.Constants;

public class AprilTagVisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  public static final Pose3d[] cameraPoses =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new Pose3d[] {
              new Pose3d(
                  Units.inchesToMeters(9.735),
                  Units.inchesToMeters(9.974),
                  Units.inchesToMeters(8.837),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))),
              new Pose3d(
                  Units.inchesToMeters(9.735),
                  Units.inchesToMeters(-9.974),
                  Units.inchesToMeters(8.837),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30.0))))
            };
        default -> new Pose3d[] {};
      };

  public static final String[] instanceNames =
      switch (Constants.getRobot()) {
        case DEVBOT -> new String[] {"northstar_0", "northstar_1"};
        default -> new String[] {};
      };

  public static final String[] cameraIds =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new String[] {
              "/dev/v4l/by-path/platform-fc800000.usb-usb-0:1:1.0-video-index0",
              "/dev/v4l/by-path/platform-fc880000.usb-usb-0:1:1.0-video-index0"
            };
        default -> new String[] {};
      };
}
