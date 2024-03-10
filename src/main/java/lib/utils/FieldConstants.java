package lib.utils;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

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
  private FieldConstants() {}
  public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
  public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);
  public static final double WING_X = Units.inchesToMeters(229.201);
  public static final double PODIUM_X = Units.inchesToMeters(126.75);
  public static final double STARTING_LINE_X = Units.inchesToMeters(74.111);

  public static final Translation2d AMP_CENTER =
      new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

  public static final Pose2d AMP_LINEUP =
    new Pose2d(
        AMP_CENTER.getX() - Units.inchesToMeters(10),
        AMP_CENTER.getY(),
        Rotation2d.fromDegrees(90)
    );

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static final double CENTERLINE_X = FIELD_LENGTH / 2.0;

    // need to update
    public static final double CENTERLINE_FIRST_Y = Units.inchesToMeters(29.638);
    public static final double CENTERLINE_SEPARATION_Y = Units.inchesToMeters(66);
    public static final double SPIKE_X = Units.inchesToMeters(114);
    // need
    public static final double SPIKE_FIRST_Y = Units.inchesToMeters(161.638);
    public static final double SPIKE_SEPARATION_Y = Units.inchesToMeters(57);

    public static final Translation2d[] CENTERLINE_TRANSLATIONS = new Translation2d[5];
    public static final Translation2d[] SPIKE_TRANSLATIONS = new Translation2d[3];

    static {
      for (int i = 0; i < CENTERLINE_TRANSLATIONS.length; i++) {
        CENTERLINE_TRANSLATIONS[i] =
            new Translation2d(CENTERLINE_X, CENTERLINE_FIRST_Y + (i * CENTERLINE_SEPARATION_Y));
      }
    }

    static {
      for (int i = 0; i < SPIKE_TRANSLATIONS.length; i++) {
        SPIKE_TRANSLATIONS[i] = new Translation2d(SPIKE_X, SPIKE_FIRST_Y + (i * SPIKE_SEPARATION_Y));
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class Speaker {

    /** Center of the speaker opening (blue alliance) */
    public static final Pose2d CENTER_SPEAKER_OPENING =
        new Pose2d(0.0, FIELD_WIDTH - Units.inchesToMeters(104.0), new Rotation2d());
  }

  // corners (blue alliance origin)
  public static final Translation3d TOP_RIGHT_SPEAKER =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(238.815),
          Units.inchesToMeters(83.091));

  public static final Translation3d TOP_LEFT_SPEAKER =
      new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(197.765),
          Units.inchesToMeters(83.091));

  public static final Translation3d BOTTOM_RIGHT_SPEAKER =
      new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
  public static final Translation3d BOTTOM_LEFT_SPEAKER =
      new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

  public static final Translation3d CENTER_SPEAKER =
      new Translation3d(
          (Units.inchesToMeters(18.055) / 2.0),
          Units.inchesToMeters((238.815 + 197.765) / 2.0),
          (Units.inchesToMeters(83.091 + 78.324) / 2.0) - Units.inchesToMeters(1.0));

  public static final double APRIL_TAG_WIDTH = Units.inchesToMeters(6.50);
  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

  static {
    try {
      APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
