package lib.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmPose;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class AimbotUtils {

  private static final InterpolatingDoubleTreeMap m_angleLerpTable = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_leftSpeedLerpTable = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_rightSpeedLerpTable = new InterpolatingDoubleTreeMap();

  private static final LoggedDashboardNumber m_offsetNudge =
      new LoggedDashboardNumber("Wrist Angle Nudge", 0.01);

  private static final double Y_TARGET = 0.2;

  static {
    // angle measurements, meters -> degrees
    m_angleLerpTable.put(Units.inchesToMeters(18.0), 57.0 + 3.0);
    m_angleLerpTable.put(Units.inchesToMeters(46.0), 45.0 + 3.0);
    m_angleLerpTable.put(Units.inchesToMeters(97.0), 33.5 + 2.5);
    m_angleLerpTable.put(Units.inchesToMeters(116.0), 29.75 + 2.5);
    m_angleLerpTable.put(Units.inchesToMeters(155.0), 24.65 + 1.5);

//    m_angleLerpTable.put(Units.inchesToMeters(229.0), 0.0);

    //flywheel measurements, meters -> RPM
    m_leftSpeedLerpTable.put(Units.inchesToMeters(18.0), 3600.0);
    m_leftSpeedLerpTable.put(Units.inchesToMeters(46.0), 3700.0);
    m_leftSpeedLerpTable.put(Units.inchesToMeters(97.0), 3800.0);
    m_leftSpeedLerpTable.put(Units.inchesToMeters(116.0), 4000.0);
    m_leftSpeedLerpTable.put(Units.inchesToMeters(155.0), 5000.0);
//    m_leftSpeedLerpTable.put(Units.inchesToMeters(229.0), 0.0);

    m_rightSpeedLerpTable.put(Units.inchesToMeters(18.0), 3600.0);
    m_rightSpeedLerpTable.put(Units.inchesToMeters(46.0), 3600.0);
    m_rightSpeedLerpTable.put(Units.inchesToMeters(97.0), 3700.0);
    m_rightSpeedLerpTable.put(Units.inchesToMeters(116.0), 3800.0);
    m_rightSpeedLerpTable.put(Units.inchesToMeters(155.0), 4600.0);
//    m_rightSpeedLerpTable.put(Units.inchesToMeters(229.0), 0.0);
  }

  /** Linear interpolation tables for aiming */
  public static double getWristAngle(double distance) {
    double angle = 49.319 + (1.427 * Y_TARGET) + (-0.10599 * distance);
    if (90.0 >= distance && distance > 55.0) {
      return angle;
    } else if (150.0 >= distance && distance > 90.0) {
      return angle - 1.8;
    } else if (195.0 >= distance && distance > 150.0) {
      return angle - 3.75;
    } else if (distance > 200.0) {
      return angle;
    } else if (distance > 195.0) {
      return angle - 4.25;
    } else {
      return 55.0;
    }
  }

  public static double getLeftSpeed(double distance) {
    return distance >= 75.0 ? 4000 : 3750;
  }

  public static double getRightSpeed(double distance) {
    return distance >= 75.0 ? 4750 : 4500;
  }

  /** Gets the distance from the drivebase to the speaker in meters */
  public static double getDistanceFromSpeaker(Pose2d drivePose) {
    return AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER).toTranslation2d()
        .getDistance(drivePose.getTranslation());
  }

  /** Gets the angle the drivebase should be at with a default of the speaker */
  public static Rotation2d getDrivebaseAimingAngle(Pose2d drivePose) {
    return getDrivebaseAimingAngle(drivePose, FieldConstants.CENTER_SPEAKER);
  }

  /** Gets the angle the drivebase should be at to aim at the speaker */
  public static Rotation2d getDrivebaseAimingAngle(Pose2d drivePose, Translation3d target) {
    Transform3d robotToPoint =
        new Transform3d(
            new Pose3d(new Pose2d(drivePose.getTranslation(), new Rotation2d())),
            new Pose3d(AllianceFlipUtil.apply(target), new Rotation3d()));

    return Rotation2d.fromRadians(
        MathUtil.angleModulus(
            Math.PI * 2 - (Math.atan2(robotToPoint.getX(), robotToPoint.getY()))
                + Units.degreesToRadians(90)
        ));
  }

  /** Gets the top point of the shooter for checking limits*/
  public static Translation2d calculateArmPosition(double armAngle, double wristAngle) {
    return ArmConstants.PIVOT_JOINT_TRANSLATION
            // translate the length + direction of the arm
            .plus(new Translation2d(ArmConstants.ARM_LENGTH_METERS,
                    Rotation2d.fromDegrees(armAngle)))
            // translate the length + direction of the wrist
            .plus(new Translation2d(ArmConstants.WRIST_LENGTH_METERS,
                    Rotation2d.fromDegrees(360)
                            .minus(Rotation2d.fromDegrees(wristAngle))));
  }

  /** Gets the transformation of the shooter relative to the drive base */
  public static Transform3d getShooterTransformation(double armAngle) {
    return ArmConstants.PIVOT_TRANSLATION_METERS.plus(
            // Add the movement of the arm
            GeomUtils.translationToTransform(new Translation3d(
                    ArmConstants.ARM_LENGTH_METERS,
                    new Rotation3d(0.0, Units.degreesToRadians(armAngle + 180), 0.0)
            ))
    );
  }

  public static ArmPose aimbotCalculate(Pose3d robotPose, double armAngle) {
    // the position to target
    Pose3d speakerPose = new Pose3d(AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER), new Rotation3d());
    Translation2d speakerPoseGround = speakerPose.getTranslation().toTranslation2d();

    // find where the shooter is at relative to the robot
    Pose3d shooterPivotPose = robotPose.plus(getShooterTransformation(armAngle));
    // find the transformation from the shooter to the speaker
    Transform3d robotToSpeaker =
            new Transform3d(shooterPivotPose, speakerPose);

//    Logger.recordOutput("Arm/Speaker Pose", speakerPose);
//    Logger.recordOutput("Arm/Shooter to Speaker", robotPose.plus(robotToSpeaker));

    // find the distance on the ground to the speaker
    double groundDistance = robotPose.getTranslation().toTranslation2d().getDistance(speakerPoseGround);

    double desiredWristAngle = Units.radiansToDegrees(Math.atan(robotToSpeaker.getZ()/groundDistance));
    desiredWristAngle = desiredWristAngle + (90 - desiredWristAngle) * 0.06;
//    Logger.recordOutput("Arm/ Wrist Aimbot Raw", desiredWristAngle);

    double safeArmAngle = ArmConstants.WRIST_ARM_GAP.getValue() - desiredWristAngle;
    safeArmAngle = safeArmAngle >= 0 ? safeArmAngle : 0;

    return new ArmPose(safeArmAngle, desiredWristAngle);
  }
}
