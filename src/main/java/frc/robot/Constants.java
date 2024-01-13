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
import com.gos.lib.rev.swerve.RevSwerveModuleConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units.*;
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
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {

    // module constants
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    public static final ModuleConstants FL_MOD_CONSTANTS = new ModuleConstants(
            0,
            new int[]{0, 1, 2},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            0,
            true,
            ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants FR_MOD_CONSTANTS = new ModuleConstants(
            0,
            new int[]{3, 4, 5},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            0,
            true,
            ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants BL_MOD_CONSTANTS = new ModuleConstants(
            0,
            new int[]{6, 7, 8},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            0,
            false,
            ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants BR_MOD_CONSTANTS = new ModuleConstants(
            0,
            new int[]{9, 10, 11},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            0,
            false,
            ModuleConstants.GearRatios.L3
    );
  }
}
