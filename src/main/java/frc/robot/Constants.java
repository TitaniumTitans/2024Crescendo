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

import com.fasterxml.jackson.core.SerializableString;
import com.gos.lib.properties.GosBooleanProperty;
import com.gos.lib.properties.GosDoubleProperty;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.drive.module.ModuleConstants;

import java.util.List;

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

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {
    public static final GosDoubleProperty TURNING_SPEED =
            new GosDoubleProperty(false, "Drive/Turning Speed", 0.75);
    public static final GosBooleanProperty HOLD_HEADING =
            new GosBooleanProperty(false, "Drive/Hold Heading", false);

    private DriveConstants() {
      throw new IllegalStateException("Constants class should not be constructed");
    }

    public static final int PIGEON_ID = 13;

    // module constants
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0 - (1.0 / 8.0));

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.1);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(18.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final GosBooleanProperty USE_DAVID_DRIVE
        = new GosBooleanProperty(false, "Drive/Use David Drive", false);

    // kV, kS, kA in order
    public static final double[] DRIVE_FF_GAINS = new double[]{0.06, 0.35, 0.0};
    // kP, kI, kD in order
    public static final double[] DRIVE_FB_GAINS = new double[]{0.08, 0.0, 0.0};
    // kP, kI, kD in order
    public static final double[] TURN_FB_GAINS = new double[]{47.0, 0.0, 0.0};

    public static final Transform3d LEFT_CAMERA_TRANSFORMATION = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(11.0351 + 1.25), // 11.0351
            Units.inchesToMeters(10.023204 - 2.0), // 10.023204
            Units.inchesToMeters(7.1374 - 2.0)), // 4.1374
        new Rotation3d(
            Units.degreesToRadians(0.0 + 0.5),
            Units.degreesToRadians(-30.0 + 0.75), // -120.0 + 91.0
            Units.degreesToRadians(-14.7 + 6.0)) // 165.3224 + 180
    );

    public static final Transform3d RIGHT_CAMERA_TRANSFORMATION = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(11.0351 - 1.25), //11.0351
            Units.inchesToMeters(-10.023204 - 3.5), //-10.023204
            Units.inchesToMeters(7.1374 - 2.0)), // 7.1374
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-30.0 - 1.5), // -30.0 - 1
            Units.degreesToRadians(14.7 - 11.0)) // 165.3224)
    );

    public static final Transform3d INTAKE_CAMERA_TRANSFORMATION = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-16.5), //11.0351
            Units.inchesToMeters(-9.25), //-10.023204
            Units.inchesToMeters(10.0)), // 7.1374
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-3.0), // -30.0 - 1
            Units.degreesToRadians(180.0)) // 165.3224)
    );

    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
        MAX_LINEAR_SPEED * 0.85,
        MAX_LINEAR_SPEED * 0.65,
        MAX_ANGULAR_SPEED * 0.85,
        MAX_ANGULAR_SPEED * 0.65
    );

    public static final HolonomicPathFollowerConfig HOLONOMIC_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0), new PIDConstants(5.5, 0.0),
        DriveConstants.MAX_LINEAR_SPEED * 0.5, DriveConstants.DRIVE_BASE_RADIUS, new ReplanningConfig());

    public static final ModuleConstants FL_MOD_CONSTANTS = new ModuleConstants(
        0,
        new int[]{1, 2, 3}, // drive, turn, encoder
        DRIVE_FF_GAINS,
        DRIVE_FB_GAINS,
        TURN_FB_GAINS,
        Units.rotationsToDegrees(-0.039795) + 180, // offset 0.457764
        true, // inversion
        ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants FR_MOD_CONSTANTS = new ModuleConstants(
        1,
        new int[]{4, 5, 6}, // drive, turn, encoder
        DRIVE_FF_GAINS,
        DRIVE_FB_GAINS,
        TURN_FB_GAINS,
        Units.rotationsToDegrees(-0.297363) + 180,
        true,
        ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants BL_MOD_CONSTANTS = new ModuleConstants(
        2,
        new int[]{7, 8, 9}, // drive, turn, encoder
        DRIVE_FF_GAINS,
        DRIVE_FB_GAINS,
        TURN_FB_GAINS,
        Units.rotationsToDegrees(0.333740) + 180,
        true,
        ModuleConstants.GearRatios.L3
    );

    public static final ModuleConstants BR_MOD_CONSTANTS = new ModuleConstants(
        3,
        new int[]{10, 11, 12}, // drive, turn, encoder
        DRIVE_FF_GAINS,
        DRIVE_FB_GAINS,
        TURN_FB_GAINS,
        Units.rotationsToDegrees(0.235107) + 180,
        true,
        ModuleConstants.GearRatios.L3
    );
  }


  public static class ArmConstants {
    private ArmConstants() {}

    public static final int WRIST_MASTER_ID = 21;
    public static final int WRIST_FOLLOWER_ID = 22;
    public static final int WRIST_ENCODER_ID = 23;
    public static final int ARM_MASTER_ID = 19;
    public static final int ARM_FOLLOWER_ID = 18;
    public static final int ARM_ENCODER_ID = 20;

    /* Because the absolute encoders are on a 2/1 ratio, we have to move our offset down a little into a rotation lower
    * than the lowest point the arm and wrist will move to , and then compensate for that in our encoder reset code */
    public static final double OFFSET_NUDGE = 45;
    public static final double ARM_OFFSET = -0.123779 + Units.degreesToRotations(OFFSET_NUDGE);
    public static final double WRIST_OFFSET = 0.000000 + Units.degreesToRotations(OFFSET_NUDGE);
    public static final double ARM_SENSOR_MECHANISM_RATIO =
        (56.0 / 12.0) * (66.0 / 18.0) * (80.0 / 18.0) * (64.0 / 24.0);
    public static final double ARM_CANCODER_MECHANISM_RATIO = (26.0 / 36.0) * (64.0 / 24.0);

    // the pulley on the encoder is a 1:1
    public static final double WRIST_SENSOR_MECHANISM_RATIO =
        (56.0 / 12.0) * (66.0 / 18.0) * (80.0 / 18.0) * (48.0 / 24.0);
    public static final double WRIST_CANCODER_MECHANISM_RATIO = (48.0 / 24.0);

    public static final double WRIST_KP = 350.0;
    public static final double WRIST_KI = 6.0;
    public static final double WRIST_KD = 3.0;
    public static final double WRIST_KS = 0.375;
    public static final double WRIST_KV = 0.0;
    public static final double WRIST_KG = 0.35;

    public static final double ARM_KP = 350.0;
    public static final double ARM_KI = 6.0;
    public static final double ARM_KD = 3.0;
    public static final double ARM_KS = 0.375;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KG = 0.375;

    public static final GosDoubleProperty WRIST_LOWER_LIMIT =
        new GosDoubleProperty(true, "Arm/WristLowerLimit", 0);
    public static final GosDoubleProperty WRIST_UPPER_LIMIT =
        new GosDoubleProperty(true, "Arm/WristUpperLimit", 180);

    public static final GosDoubleProperty ARM_LOWER_LIMIT =
        new GosDoubleProperty(true, "Arm/ArmLowerLimit", -9);
    public static final GosDoubleProperty ARM_UPPER_LIMIT =
        new GosDoubleProperty(true, "Arm/ArmUpperLimit", 180);

    public static final GosDoubleProperty WRIST_ARM_GAP =
        new GosDoubleProperty(true, "Arm/Wrist Gap", 10);

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
    public static final ArmPose PASS_SETPOINT = new ArmPose("ArmPoses/Pass Setpoint", false, 45, 55);
    public static final ArmPose TRAP_PREPARE = new ArmPose(92.0, 145.0);
    public static final ArmPose TRAP_SCORE = new ArmPose(47.0, 120.5);

    private ArmSetpoints() {
      throw new IllegalStateException("Static classes should not be constructed");
    }

    public static final ArmPose AMP_INTERMEDIATE = new ArmPose("ArmPoses/Amp Intermediate", false, 60.0, 145.0);

    public static final ArmPose STOW_SETPOINT = new
        ArmPose("ArmPoses/Stow", true, 0.0, 35.0);
    public static final ArmPose INTAKE_SETPOINT =
        new ArmPose("ArmPoses/Intake", true, -5.75, 45.0);
    public static final ArmPose AMP_SETPOINT =
        new ArmPose("ArmPoses/Amp", true, 94.0, 145.0);

    public static final ArmPose STATIC_SHOOTER = new ArmPose("ArmPoses/ShooterTesting", false, 0.0, 55.0);

    public static final GosDoubleProperty WRIST_ANGLE = new GosDoubleProperty(false, "Wrist Angle", 45.0);

    public static final Trajectory STOW_AMP_TRAJ;
    public static final Trajectory AMP_STOW_TRAJ;

    static {
      // kinda janky(?) spline generation
      // use Pose2d x for arm angle and y for wrist angle, ignore heading
      var stowPose = new Pose2d(0.0, 45.0, new Rotation2d());
      var ampPose = new Pose2d(94.0, 145.0, new Rotation2d());
      var ampIntermediatePose = List.of(new Pose2d(60.0, 145.0, new Rotation2d()).getTranslation());

      var trajConfig = new TrajectoryConfig(Units.degreesToRotations(30), Units.degreesToRotations(30));

      STOW_AMP_TRAJ = TrajectoryGenerator.generateTrajectory(
              stowPose,
              ampIntermediatePose,
              ampPose,
              trajConfig
      );

      AMP_STOW_TRAJ = TrajectoryGenerator.generateTrajectory(
              ampPose,
              ampIntermediatePose,
              stowPose,
              trajConfig
      );
    }
  }

  public static class ShooterConstants {
    public static final GosDoubleProperty ACCEL_COMP_FACTOR =
        new GosDoubleProperty(false, "Shooter/Acceleration Compensation", 0.100);

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

    public static final double SHOOTER_KP = 0.015;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.00675;
    public static final double SHOOTER_KF = 0.000152;
    public static final double SHOOTER_KS = 0.21963;
    public static final double SHOOTER_KV = 0.114541;

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

    public static final double CLIMBER_KP = 24.0;
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
