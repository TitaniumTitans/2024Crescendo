package lib.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmPose;
import org.littletonrobotics.junction.Logger;

public class AimbotUtils {

  private static final InterpolatingDoubleTreeMap m_angleLerpTable = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_speedLerpTable = new InterpolatingDoubleTreeMap();

  static {
    // angle measurements, meters -> degrees
    m_angleLerpTable.put(Units.inchesToMeters(18.0), 50.0);
    m_angleLerpTable.put(Units.inchesToMeters(46.0), 40.0);
    m_angleLerpTable.put(Units.inchesToMeters(97.0), 31.5);

    //flywheel measurements, meters -> RPM
    m_speedLerpTable.put(Units.inchesToMeters(18.0), 3600.0);
    m_speedLerpTable.put(Units.inchesToMeters(46.0), 3600.0);
    m_speedLerpTable.put(Units.inchesToMeters(97.0), 3600.0);
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
