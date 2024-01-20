package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleConstants {

    public enum GearRatios {
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
    public final double kDriveGearRatio;
    public final double kTurningGearRatio;

    public final double kWheelRadiusMeters;
    public final double kWheelCurcumferenceMeters;

    public final boolean kTurnMotorInverted;
    public final boolean kDriveMotorInverted;
    public final boolean kEncoderInverted;

    public final Rotation2d kEncoderOffset;

    public final int kModuleIndex;

    // can ID's
    public final int kDriveMotorId;
    public final int kTurnMotorId;
    public final int kEncoderId;

    // Drive loop gains
    public final double kDriveKv;
    public final double kDriveKs;
    public final double kDriveKa;

    public final double kDriveKp;
    public final double kDriveKi;
    public final double kDriveKd;

    // Turning loop gains
    public final double kTurnKp;
    public final double kTurnKi;
    public final double kTurnKd;

    public static final ModuleConstants BLANK_CONSTANTS = new ModuleConstants(
            0,
            new int[]{0, 0, 0},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            new double[]{0.0, 0.0, 0.0},
            0.0,
            false,
            GearRatios.L1
    );

    public ModuleConstants(int id,
                           int[] ids,
                           double[] driveFF,
                           double[] driveFB,
                           double[] turnFB,
                           double offsetDegs,
                           boolean inverted,
                           GearRatios driveRatio) {

        double defaultTurnRatio = (150.0 / 7.0);
        double defaultWheelRadiusMeters = Units.inchesToMeters(2.0);

        kModuleIndex = id;

        kDriveMotorId = ids[0];
        kTurnMotorId = ids[1];
        kEncoderId = ids[2];

        kDriveKv = driveFF[0];
        kDriveKs = driveFF[1];
        kDriveKa = driveFF[2];

        kDriveKp = driveFB[0];
        kDriveKi = driveFB[1];
        kDriveKd = driveFB[2];

        kTurnKp = turnFB[0];
        kTurnKi = turnFB[1];
        kTurnKd = turnFB[2];

        kDriveMotorInverted = !inverted;
        kTurnMotorInverted = inverted;
        kEncoderInverted = false;

        kEncoderOffset = Rotation2d.fromDegrees(offsetDegs);

        kDriveGearRatio = driveRatio.ratio;
        kTurningGearRatio = defaultTurnRatio;

        kWheelRadiusMeters = defaultWheelRadiusMeters;
        kWheelCurcumferenceMeters = kWheelRadiusMeters * Math.PI * 2;
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
        kModuleIndex = id;

        kDriveMotorId = ids[0];
        kTurnMotorId = ids[1];
        kEncoderId = ids[2];

        kDriveKv = driveFF[0];
        kDriveKs = driveFF[1];
        kDriveKa = driveFF[2];

        kDriveKp = driveFB[0];
        kDriveKi = driveFB[1];
        kDriveKd = driveFB[2];

        kTurnKp = turnFB[0];
        kTurnKi = turnFB[1];
        kTurnKd = turnFB[2];

        kDriveMotorInverted = inversions[0];
        kTurnMotorInverted = inversions[1];
        kEncoderInverted = false;

        kEncoderOffset = Rotation2d.fromDegrees(offsetDegs);

        kDriveGearRatio = driveRatio.ratio;
        kTurningGearRatio = turningRatio;

        kWheelRadiusMeters = wheelRadiusMeter;
        kWheelCurcumferenceMeters = kWheelRadiusMeters * Math.PI * 2;
    }
}
