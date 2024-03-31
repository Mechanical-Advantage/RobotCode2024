// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2024.FieldConstants;
import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final double shotSpeed = 15.0; // Meters per sec
  private static final double ejectSpeed = 2.0; // Meters per sec
  @Setter private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  @Setter private static Supplier<Rotation2d> armAngleSupplier = Rotation2d::new;
  private static final List<Translation2d> autoNotes = new ArrayList<>();
  @Setter @Getter private static boolean hasNote = false;

  /** Show all staged notes for alliance */
  public static void showAutoNotes() {
    if (autoNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes", new Pose3d[] {});
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = autoNotes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public static void clearAutoNotes() {
    autoNotes.clear();
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetAutoNotes() {
    clearAutoNotes();
    for (int i = 0; i < 3; i++) {
      autoNotes.add(AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[i]));
    }
    for (int i = 0; i < 5; i++) {
      autoNotes.add(
          AllianceFlipUtil.apply(FieldConstants.StagingLocations.centerlineTranslations[i]));
    }
  }

  /**
   * Intakes staged note.
   *
   * @param index of staged note (0-2) spikes, (3-7) centerlines
   */
  public static void intakeAutoNote(int index) {
    if (autoNotes.get(index) != null) {
      setHasNote(true);
    }
    autoNotes.set(index, null);
  }

  /** Shows the currently held note if there is one */
  public static void showHeldNotes() {
    if (hasNote) {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {getIndexerPose3d()});
    } else {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {});
    }
  }

  /** Shoots note from middle of arm to speaker */
  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  final Pose3d startPose = getIndexerPose3d();
                  final Pose3d endPose =
                      new Pose3d(
                              AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening),
                              startPose.getRotation())
                          .transformBy(new Transform3d(0.4, 0.0, 0.0, new Rotation3d()));
                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  public static Command eject() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  final Pose3d startPose = getIndexerPose3d();
                  System.out.println(getIndexerPose3d().getZ());
                  final Pose3d endPose =
                      startPose.transformBy(
                          new Transform3d(2, 0, -1 + startPose.getZ(), new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/EjectNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  private static Pose3d getIndexerPose3d() {
    Transform3d indexerTransform =
        new Transform3d(
                ArmConstants.armOrigin.getX(),
                0.0,
                ArmConstants.armOrigin.getY(),
                new Rotation3d(0.0, -armAngleSupplier.get().getRadians(), 0.0))
            .plus(new Transform3d(ArmConstants.armLength * 0.35, 0.0, 0.0, new Rotation3d()));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }
}
