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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import com.gos.lib.properties.GosDoubleProperty;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.units.Unit;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmSubsystem;
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

  public static final Mode currentMode = Mode.REAL;

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

    public static final int PIGEON_ID = 13;

    // module constants
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(18.0);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(18.6);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(18.6);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    // kV, kS, kA in order
    public static final double[] DRIVE_FF_GAINS = new double[]{0.13, 0.1, 0.0};
    // kP, kI, kD in order
    public static final double[] DRIVE_FB_GAINS = new double[]{0.314, 0.0, 0.0};
    // kP, kI, kD in order
    public static final double[] TURN_FB_GAINS = new double[]{43.982, 0.0, 0.0};

//    public static final Transform3d RIGHT_CAMERA_TRANSFORMATION = new Transform3d(
//        new Translation3d(Units.inchesToMeters(10.5), Units.inchesToMeters(8.5), Units.inchesToMeters(6)),
//        new Rotation3d(0.0, Units.degreesToRadians(50), Units.degreesToRadians(-18))
//    );
//    public static final Transform3d LEFT_CAMERA_TRANSFORMATION = new Transform3d(
//                new Translation3d(Units.inchesToMeters(10.5), Units.inchesToMeters(-8.5), Units.inchesToMeters(6)),
//        new Rotation3d(0.0, Units.degreesToRadians(50), Units.degreesToRadians(18))
//        );

    public static final Transform3d LEFT_CAMERA_TRANSFORMATION = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.25), Units.inchesToMeters(9.0), Units.inchesToMeters(6.0)),
        new Rotation3d(Units.degreesToRadians(5.0), Units.degreesToRadians(-28.125), Units.degreesToRadians(35.0 + 180))
    );

    public static final Transform3d RIGHT_CAMERA_TRANSFORMATION = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.25), Units.inchesToMeters(-9.0), Units.inchesToMeters(6.0)),
        new Rotation3d(Units.degreesToRadians(2.0), Units.degreesToRadians(-26.0), Units.degreesToRadians(-35.0 - 180))
    );

    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
        Units.radiansToDegrees(MAX_LINEAR_SPEED),
        Units.radiansToDegrees(MAX_LINEAR_SPEED),
        MAX_ANGULAR_SPEED,
        MAX_ANGULAR_SPEED
    );

    public static final HolonomicPathFollowerConfig HOLONOMIC_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0),new PIDConstants(5.0),
        DriveConstants.MAX_LINEAR_SPEED, DriveConstants.DRIVE_BASE_RADIUS, new ReplanningConfig());

    public static final ModuleConstants FL_MOD_CONSTANTS = new ModuleConstants(
            0,
            new int[]{1, 2, 3}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(0.472168), // offset 0.457764
            true, // inversion
            ModuleConstants.GearRatios.L3_KRAKEN
    );

    public static final ModuleConstants FR_MOD_CONSTANTS = new ModuleConstants(
            1,
            new int[]{4, 5, 6}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(0.046143),
            true,
            ModuleConstants.GearRatios.L3_KRAKEN
    );

    public static final ModuleConstants BL_MOD_CONSTANTS = new ModuleConstants(
            2,
            new int[]{7, 8, 9}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(-0.073730),
            true,
            ModuleConstants.GearRatios.L3_KRAKEN
    );

    public static final ModuleConstants BR_MOD_CONSTANTS = new ModuleConstants(
            3,
            new int[]{10, 11, 12}, // drive, turn, encoder
            DRIVE_FF_GAINS,
            DRIVE_FB_GAINS,
            TURN_FB_GAINS,
            Units.rotationsToDegrees(0.404297),
            true,
            ModuleConstants.GearRatios.L3_KRAKEN
    );
  }


  public static class ArmConstants {
    private ArmConstants() {}

    public static final int WRIST_MASTER_ID = 21;
    public static final int WRIST_FOLLOWER_ID = 22;
    public static final int WRIST_ENCODER_ID = 23;
    public static final int ARM_MASTER_ID = 18;
    public static final int ARM_FOLLOWER_ID = 19;
    public static final int ARM_ENCODER_ID = 20;

    /* Because the absolute encoders are on a 2/1 ratio, we have to move our offset down a little into a rotation lower
    * than the lowest point the arm and wrist will move to , and then compensate for that in our encoder reset code */
    public static final double OFFSET_NUDGE = 45;
    public static final double ARM_OFFSET = -0.604004 + Units.degreesToRotations(OFFSET_NUDGE);
    public static final double WRIST_OFFSET = -0.108643 + Units.degreesToRotations(OFFSET_NUDGE);
    public static final double ARM_SENSOR_MECHANISM_RATIO = (56.0 / 12.0) * (66.0 / 18.0) * (80.0 / 18.0) * (64.0 / 24.0);
    public static final double ARM_CANCODER_MECHANISM_RATIO = (26.0 / 36.0) * (64.0 / 24.0);

    // the pulley on the encoder is a 1:1
    public static final double WRIST_SENSOR_MECHANISM_RATIO = (56.0 / 12.0) * (66.0 / 18.0) * (80.0 / 18.0) * (48.0 / 24.0);
    public static final double WRIST_CANCODER_MECHANISM_RATIO = (48.0 / 24.0);

    public static final double WRIST_KP = 108.0;
    public static final double WRIST_KI = 3.0;
    public static final double WRIST_KD = 3.0;
    public static final double WRIST_KS = 0.375;
    public static final double WRIST_KV = 0.0;
    public static final double WRIST_KG = 0.35;

    public static final double ARM_KP = 72.0;
    public static final double ARM_KI = 6.0;
    public static final double ARM_KD = 3.0;
    public static final double ARM_KS = 0.375;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KG = 0.375;

    public static final GosDoubleProperty WRIST_LOWER_LIMIT =
        new GosDoubleProperty(false, "Arm/WristLowerLimit", 30);
    public static final GosDoubleProperty WRIST_UPPER_LIMIT =
        new GosDoubleProperty(false, "Arm/WristUpperLimit", 30);

    public static final GosDoubleProperty ARM_LOWER_LIMIT =
        new GosDoubleProperty(false, "Arm/ArmLowerLimit", 30);
    public static final GosDoubleProperty ARM_UPPER_LIMIT =
        new GosDoubleProperty(false, "Arm/ArmUpperLimit", 30);

    public static final GosDoubleProperty WRIST_ARM_GAP =
        new GosDoubleProperty(false, "Arm/Wrist Gap", 20);

    public static final Translation2d PIVOT_JOINT_TRANSLATION =
        new Translation2d(Units.inchesToMeters(9.27),
            Units.inchesToMeters(12.56));

    public static final Transform3d PIVOT_TRANSLATION_METERS =
            new Transform3d(Units.inchesToMeters(9.27),
                    0.0,
                    Units.inchesToMeters(12.56),
                    new Rotation3d());

    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(22.01);
    public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(14.5);
  }

  public static class ArmSetpoints {
    public static final ArmPose AMP_INTERMEDIATE = new ArmPose("ArmPoses/Amp Intermediate", false, 60.0, 145.0);

    private ArmSetpoints() {
      throw new IllegalStateException("Static classes should not be constructed");
    }

    public static final ArmPose STOW_SETPOINT = new
        ArmPose("ArmPoses/Stow", false, 0.0, 45.0);
    public static final ArmPose INTAKE_SETPOINT =
        new ArmPose("ArmPoses/Intake", false, 0.0, 35.0);
    public static final ArmPose AMP_SETPOINT =
        new ArmPose("ArmPoses/Amp", false, 90.0, 135.0);

    public static final GosDoubleProperty WRIST_ANGLE = new GosDoubleProperty(false, "Wrist Angle", 45.0);
  }

  public static class ShooterConstants {
    private ShooterConstants() {
      throw new IllegalStateException("Static classes should not be constructed");
    }


    public static final int TOP_LEFT_ID = 25;
    public static final int TOP_RIGHT_ID = 26;
    public static final int BOTTOM_LEFT_ID = 28;
    public static final int BOTTOM_RIGHT_ID = 29;
    public static final int KICKER_ID = 24;
    public static final int INTAKE_ID = 14;
    public static final int INDEXER_ID = 15;

    public static final double SHOOTER_KP = 0.19623;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.005;
    public static final double SHOOTER_KF = 0.000152;
    public static final double SHOOTER_KS = 0.21963;
    public static final double SHOOTER_KV = 0.13041;

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


    public static final int LEFT_CLIMBER_ID = 16;
    public static final int RIGHT_CLIMBER_ID = 17;

    public static final double CLIMBER_KP = 0.0;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.0;

    public static final double CLIMBER_KV = 0.0;
    public static final double CLIMBER_KS = 0.0;
    public static final double CLIMBER_KA = 0.0;
    public static final double CLIMBER_KG = 0.0;

    public static final double CLIMBER_GEAR_RATIO = 0.0;
    public static final double CLIMBER_CONVERSION_FACTOR_INCHES = 0.0;
  }
}
