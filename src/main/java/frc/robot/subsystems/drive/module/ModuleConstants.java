package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public record ModuleConstants(
    // Physical constants
    double DRIVE_GEAR_RATIO, double TURNING_GEAR_RATIO,
    double WHEEL_RADIUS_METERS, double WHEEL_CURCUMFERENCE_METERS,
    boolean TURN_MOTOR_INVERTED, boolean DRIVE_MOTOR_INVERTED, boolean ENCODER_INVERTED,
    Rotation2d ENCODER_OFFSET,
    int MODULE_INDEX,
    // can ID's
    int DRIVE_MOTOR_ID, int TURN_MOTOR_ID, int ENCODER_ID,
    // Drive loop gains
    double DRIVE_KV, double DRIVE_KS, double DRIVE_KA,
    double DRIVE_KP, double DRIVE_KI, double DRIVE_KD,
    // Turning loop gains
    double TURN_KP, double TURN_KI, double TURN_KD) {
  public static final double DEFAULT_WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  public static final ModuleConstants BLANK_CONSTANTS = new ModuleConstants(
          0,
          new int[] {0, 0, 0},
          new double[] {0.0, 0.0, 0.0},
          new double[] {0.0, 0.0, 0.0},
          new double[] {0.0, 0.0, 0.0},
          0.0,
          false,
          GearRatios.L3
  );

  public ModuleConstants() {
    this (0, new int[] {0, 0, 0},
            new double[] {0.0, 0.0, 0.0},
            new double[] {0.0, 0.0, 0.0},
            new double[] {0.0, 0.0, 0.0},
    0.0, false, GearRatios.L3);
  }

  public enum GearRatios {
    TURN(150.0 / 7.0),
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0));

    GearRatios(double ratio) {
      this.ratio = ratio;
    }

    public final double ratio;
    }

  public ModuleConstants(int id,
                         int[] ids,
                         double[] driveFF,
                         double[] driveFB,
                         double[] turnFB,
                         double offsetDegs,
                         boolean inverted,
                         GearRatios driveRatio) {

    this (
    driveRatio.ratio, GearRatios.TURN.ratio,
            DEFAULT_WHEEL_RADIUS_METERS,
            DEFAULT_WHEEL_RADIUS_METERS * 2 * Math.PI,
            inverted,
            !inverted,
            false,
            Rotation2d.fromDegrees(offsetDegs),
            id,
            ids[0], ids[1], ids[2],
            driveFF[0], driveFF[1], driveFF[2],
            driveFB[0], driveFB[1], driveFB[2],
            turnFB[0], turnFB[1], turnFB[2]
    );
  }
}
