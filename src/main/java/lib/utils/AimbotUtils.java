package lib.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;

public class AimbotUtils {
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
                    new Rotation3d(0.0, Units.degreesToRadians(armAngle), 0.0)
            ))
    );
  }

  public static ArmSetpoints.ArmSetpoint aimbotCalculate(Pose3d robotPose, double armAngle) {
    Pose3d speakerPose = new Pose3d(AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER), new Rotation3d());
    Pose3d shooterPivotPose = robotPose.plus(getShooterTransformation(armAngle));
    Transform3d robotToSpeaker =
            new Transform3d(shooterPivotPose.plus(getShooterTransformation(armAngle)), speakerPose);

    double groundDistance =
            Math.sqrt(Math.pow(robotToSpeaker.getX(), 2) + Math.pow(robotToSpeaker.getY(), 2));

    double desiredWristAngle = Math.atan2(groundDistance, robotToSpeaker.getZ());
    double safeArmAngle = desiredWristAngle - ArmConstants.WRIST_ARM_GAP.getValue();
    return new ArmSetpoints.ArmSetpoint(safeArmAngle, desiredWristAngle);
  }
}
