// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.util;

import edu.wpi.first.wpilibj.SerialPort;
import java.util.Arrays;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.frc2024.Constants;
import org.littletonrobotics.frc2024.Constants.Mode;
import org.littletonrobotics.frc2024.Constants.RobotType;

public class BatteryTracker {
  private static final List<RobotType> supportedRobots = List.of(RobotType.COMPBOT);
  public static final String defaultName = "0000-000";

  private static final int nameLength = 8;
  private static final byte[] scanCommand =
      new byte[] {0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, (byte) 0xab, (byte) 0xcd};
  private static final byte[] responsePrefix =
      new byte[] {0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31};
  private static final int fullResponseLength = responsePrefix.length + nameLength;

  /** -- GETTER -- Returns the name of the last scanned battery. */
  @Getter private static String name = defaultName;

  /**
   * Scans the battery. This should be called before the first loop cycle
   *
   * @param timeout The time to wait before giving up
   */
  public static String scanBattery(double timeout) {
    if (Constants.getMode() == Mode.REAL) {
      if (supportedRobots.contains(Constants.getRobot())) {
        // Only scan on supported robots and in real mode

        try (SerialPort port = new SerialPort(9600, SerialPort.Port.kUSB)) {
          port.setTimeout(timeout);
          port.setWriteBufferSize(scanCommand.length);
          port.setReadBufferSize(fullResponseLength);

          port.write(scanCommand, scanCommand.length);
          byte[] response = port.read(fullResponseLength);

          // Ensure response is correct length
          if (response.length != fullResponseLength) {
            System.out.println(
                "[BatteryTracker] Expected "
                    + fullResponseLength
                    + " bytes from scanner, got "
                    + response.length);
            return name;
          }

          // Ensure response starts with prefix
          for (int i = 0; i < responsePrefix.length; i++) {
            if (response[i] != responsePrefix[i]) {
              System.out.println("[BatteryTracker] Invalid prefix from scanner.  Got data:");
              System.out.println("[BatteryTracker] " + Arrays.toString(response));
              return name;
            }
          }

          // Read name from data
          byte[] batteryNameBytes = new byte[nameLength];
          System.arraycopy(response, responsePrefix.length, batteryNameBytes, 0, nameLength);
          name = new String(batteryNameBytes);
          System.out.println("[BatteryTracker] Scanned battery " + name);

        } catch (Exception e) {
          System.out.println("[BatteryTracker] Exception while trying to scan battery");
          e.printStackTrace();
        }
      }
    }

    return name;
  }
}
