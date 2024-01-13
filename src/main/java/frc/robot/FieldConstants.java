package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(653.22);
  public static double fieldWidth = Units.inchesToMeters(323.28);
  public static double wingX = Units.inchesToMeters(229.201);

  public static double podiumX = Units.inchesToMeters(121);
  public static double startingLineX = Units.inchesToMeters(74.111);

  public static final class StagingLocations {
    public static double centerlineX = Units.inchesToMeters(fieldLength / 2);
    // need to update
    public static double centerlineFirstY = Units.inchesToMeters(-1);
    public static double centerlineSeparationY = Units.inchesToMeters(66);
    public static double spikeX = Units.inchesToMeters(114);
    // need
    public static double spikeFirstY = Units.inchesToMeters(-1);
    public static double spikeSeparationY = Units.inchesToMeters(57);

    public static Translation2d[] centerlineTranslations = new Translation2d[5];
    public static Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }
}
