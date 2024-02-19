// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final double shotSpeed = 9.0; // Meters per sec
  @Setter private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  @Setter private static Supplier<Rotation2d> armAngleSupplier = Rotation2d::new;

  /** Shoots note from middle of arm to speaker */
  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  Transform3d indexerTransform =
                      new Transform3d(
                              ArmConstants.armOrigin.getX(),
                              0.0,
                              ArmConstants.armOrigin.getY(),
                              new Rotation3d(0.0, -armAngleSupplier.get().getRadians(), 0.0))
                          .plus(
                              new Transform3d(
                                  ArmConstants.armLength / 2.0, 0.0, 0.0, new Rotation3d()));
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
                  final Pose3d endPose =
                      new Pose3d(
                          AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening),
                          startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(() -> Logger.recordOutput("NoteVisualizer", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }
}
