// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Builder;

/**
 * Fudge factors for critical field locations which can be tuned per alliance at events.
 */
public final class FudgeFactors {
  @Builder
  public static class FudgedTransform {
    @Builder.Default private final Transform2d red = new Transform2d();

    @Builder.Default private final Transform2d blue = new Transform2d();

    public Transform2d getTransform() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? red : blue)
          .orElseGet(Transform2d::new);
    }
  }

  private FudgeFactors() {}

  public static final FudgedTransform speaker = FudgedTransform.builder().build();

  public static final FudgedTransform amp = FudgedTransform.builder().build();

  public static final FudgedTransform centerPodiumAmpChain = FudgedTransform.builder().build();

  public static final FudgedTransform centerAmpSourceChain = FudgedTransform.builder().build();

  public static final FudgedTransform centerSourcePodiumChain = FudgedTransform.builder().build();
}
