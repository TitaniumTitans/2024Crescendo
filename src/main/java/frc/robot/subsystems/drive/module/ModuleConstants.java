package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleConstants {

    public static final double DEFAULT_WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    public enum GearRatios {
        TURN(150.0 / 7.0),
        L1((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)),
        L2((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)),
        L3((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0))
        ;

        GearRatios(double ratio) {
            this.ratio = ratio;
        }

        public final double ratio;
    }
    // Physical constants
    public final double DRIVE_GEAR_RATIO;
    public final double TURNING_GEAR_RATIO;

    public final double WHEEL_RADIUS_METERS;
    public final double WHEEL_CURCUMFERENCE_METERS;

    public final boolean TURN_MOTOR_INVERTED;
    public final boolean DRIVE_MOTOR_INVERTED;
    public final boolean ENCODER_INVERTED;

    public final Rotation2d ENCODER_OFFSET;

    public final int MODULE_INDEX;

    // can ID's
    public final int DRIVE_MOTOR_ID;
    public final int TURN_MOTOR_ID;
    public final int ENCODER_ID;

    // Drive loop gains
    public final double DRIVE_KV;
    public final double DRIVE_KS;
    public final double DRIVE_KA;

    public final double DRIVE_KP;
    public final double DRIVE_KI;
    public final double DRIVE_KD;

    // Turning loop gains
    public final double TURN_KP;
    public final double TURN_KI;
    public final double TURN_KD;

    public ModuleConstants(int id,
                           int[] ids,
                           double[] driveFF,
                           double[] driveFB,
                           double[] turnFB,
                           double offsetDegs,
                           boolean inverted,
                           GearRatios driveRatio) {

        double defaultTurnRatio = GearRatios.TURN.ratio;

        MODULE_INDEX = id;

        DRIVE_MOTOR_ID = ids[0];
        TURN_MOTOR_ID = ids[1];
        ENCODER_ID = ids[2];

        DRIVE_KV = driveFF[0];
        DRIVE_KS = driveFF[1];
        DRIVE_KA = driveFF[2];

        DRIVE_KP = driveFB[0];
        DRIVE_KI = driveFB[1];
        DRIVE_KD = driveFB[2];

        TURN_KP = turnFB[0];
        TURN_KI = turnFB[1];
        TURN_KD = turnFB[2];

        DRIVE_MOTOR_INVERTED = !inverted;
        TURN_MOTOR_INVERTED = inverted;
        ENCODER_INVERTED = false;

        ENCODER_OFFSET = Rotation2d.fromDegrees(offsetDegs);

        DRIVE_GEAR_RATIO = driveRatio.ratio;
        TURNING_GEAR_RATIO = defaultTurnRatio;

        WHEEL_RADIUS_METERS = DEFAULT_WHEEL_RADIUS_METERS;
        WHEEL_CURCUMFERENCE_METERS = WHEEL_RADIUS_METERS * Math.PI * 2;
    }

    public ModuleConstants (int id,
                            int[] ids,
                            double[] driveFF,
                            double[] driveFB,
                            double[] turnFB,
                            boolean[] inversions,
                            double offsetDegs,
                            GearRatios driveRatio,
                            double turningRatio,
                            double wheelRadiusMeter) {
        MODULE_INDEX = id;

        DRIVE_MOTOR_ID = ids[0];
        TURN_MOTOR_ID = ids[1];
        ENCODER_ID = ids[2];

        DRIVE_KV = driveFF[0];
        DRIVE_KS = driveFF[1];
        DRIVE_KA = driveFF[2];

        DRIVE_KP = driveFB[0];
        DRIVE_KI = driveFB[1];
        DRIVE_KD = driveFB[2];

        TURN_KP = turnFB[0];
        TURN_KI = turnFB[1];
        TURN_KD = turnFB[2];

        DRIVE_MOTOR_INVERTED = inversions[0];
        TURN_MOTOR_INVERTED = inversions[1];
        ENCODER_INVERTED = false;

        ENCODER_OFFSET = Rotation2d.fromDegrees(offsetDegs);

        DRIVE_GEAR_RATIO = driveRatio.ratio;
        TURNING_GEAR_RATIO = turningRatio;

        WHEEL_RADIUS_METERS = wheelRadiusMeter;
        WHEEL_CURCUMFERENCE_METERS = WHEEL_RADIUS_METERS * Math.PI * 2;
    }
}
