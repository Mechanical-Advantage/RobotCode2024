// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread {
  private List<DoubleSupplier> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();
  private final Notifier notifier;
  private static SparkMaxOdometryThread instance = null;

  public static SparkMaxOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkMaxOdometryThread();
    }
    return instance;
  }

  private SparkMaxOdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkMaxOdometryThread");
    notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency);
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(100);
    Drive.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void periodic() {
    Drive.odometryLock.lock();
    Drive.timestampQueue.offer(Logger.getRealTimestamp() / 1.0e6);
    try {
      for (int i = 0; i < signals.size(); i++) {
        queues.get(i).offer(signals.get(i).getAsDouble());
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
