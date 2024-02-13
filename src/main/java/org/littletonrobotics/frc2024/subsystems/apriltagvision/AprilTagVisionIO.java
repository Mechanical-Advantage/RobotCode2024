// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.apriltagvision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  class AprilTagVisionIOInputs implements LoggableInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public double[] demoFrame = new double[] {};
    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("FrameCount", frames.length);
      for (int i = 0; i < frames.length; i++) {
        table.put("Frame/" + i, frames[i]);
      }
      table.put("DemoFrame", demoFrame);
      table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", new double[] {0.0});
      int frameCount = table.get("FrameCount", 0);
      frames = new double[frameCount][];
      for (int i = 0; i < frameCount; i++) {
        frames[i] = table.get("Frame/" + i, new double[] {});
      }
      demoFrame = table.get("DemoFrame", new double[] {});
      fps = table.get("Fps", 0);
    }
  }

  default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
