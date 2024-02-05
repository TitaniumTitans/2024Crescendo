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

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.module.ModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private Constants() {
    throw new IllegalStateException("Constants class should not be constructed");
  }
  public static final Mode currentMode = Mode.PROTO_ARM;

  public enum Mode {
    /** Running on a real robot. */
    REAL,
    PROTO_SHOOTER,
    PROTO_ARM,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {
    private DriveConstants() {
      throw new IllegalStateException("Constants class should not be constructed");
    }

    // module constants
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    // kV, kS, kA in order
    protected static final double[] DRIVE_FF_GAINS = new double[]{0.13, 0.1, 0.0};
    // kP, kI, kD in order
    protected static final double[] DRIVE_FB_GAINS = new double[]{0.05, 0.0, 0.0};
    // kP, kI, kD in order
    protected static final double[] TURN_FB_GAINS = new double[]{7.0, 0.0, 0.0};

    public static final ModuleConstants FL_MOD_CONSTANTS = new ModuleConstants(
            0,
            new int[]{0, 1, 2}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(-0.017578), // offset 0.457764
            true, // inversion
            ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants FR_MOD_CONSTANTS = new ModuleConstants(
            1,
            new int[]{3, 4, 5}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(0.201172),
            true,
            ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants BL_MOD_CONSTANTS = new ModuleConstants(
            2,
            new int[]{6, 7, 8}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(-0.017578),
            false,
            ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants BR_MOD_CONSTANTS = new ModuleConstants(
            3,
            new int[]{9, 10, 11}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(-0.263916),
            false,
            ModuleConstants.GearRatios.L3
    );
  }

  public static class ArmConstants {
    private ArmConstants(){}

    public static final double SHOULDER_BAR_LENGTH_METERS = 0.0;
    public static final double SHOOTER_BAR_LENGTH_METERS = 0.0;

    public static final Translation2d PIVOT_TRANSLATION_METERS = new Translation2d(
      0.0,
      0.0
    );
  }


  public static class ShooterConstants {
    private ShooterConstants() {
      throw new IllegalStateException("Static classes should not be constructed");
    }
    public static final int TOP_LEFT_ID = 13;
    public static final int TOP_RIGHT_ID = 14;
    public static final int BOTTOM_LEFT_ID = 15;
    public static final int BOTTOM_RIGHT_ID = 16;
    public static final int KICKER_ID = 17;

    public static final double SHOOTER_KP = 0.000061;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.005;
    public static final double SHOOTER_KF = 0.000152;

    public static final boolean TOP_LEFT_INVERTED = false;
    public static final boolean TOP_RIGHT_INVERTED = true;
    public static final boolean BOTTOM_LEFT_INVERTED = true;
    public static final boolean BOTTOM_RIGHT_INVERTED = false;
    public static final boolean KICKER_INVERTED = true;
  }

  public static class ClimberConstants {
    private ClimberConstants() {
      throw new IllegalStateException("Static class should not be constructed");
    }

    public static final int LEFT_CLIMBER_ID = 18;
    public static final int RIGHT_CLIMBER_ID = 19;

    public static final double CLIMBER_KP = 0.0;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.0;

    public static final double CLIMBER_KV = 0.0;
    public static final double CLIMBER_KS = 0.0;
    public static final double CLIMBER_KA = 0.0;

    public static final double CLIMBER_GEAR_RATIO = 0.0;
    public static final double CLIMBER_CONVERSION_FACTOR_INCHES = 0.0;
  }
}
