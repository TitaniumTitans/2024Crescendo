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
    protected static final double[] TURN_FB_GAINS = new double[]{0.1, 0.0, 0.0};

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
            Units.rotationsToDegrees(0.457764),
            true,
            ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants BR_MOD_CONSTANTS = new ModuleConstants(
            3,
            new int[]{9, 10, 11}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(-0.263916),
            true,
            ModuleConstants.GearRatios.L3
    );
  }

  public static class ArmConstants {
    private ArmConstants() {}

    public static int WRIST_ID = 0;
    public static int SHOULDER_ID = 0;

    public static double WRIST_KP = 0.0;
    public static double WRIST_KI = 0.0;
    public static double WRIST_KD = 0.0;
    public static double WRIST_KS = 0.0;
    public static double WRIST_KV = 0.0;
    public static double WRIST_KG = 0.0;

    public static double SHOULDER_KP = 0.0;
    public static double SHOULDER_KI = 0.0;
    public static double SHOULDER_KD = 0.0;
    public static double SHOULDER_KS = 0.0;
    public static double SHOULDER_KV = 0.0;
    public static double SHOULDER_KG = 0.0;
  }
}
