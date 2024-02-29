package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import lib.utils.AimbotUtils;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d arm;
  private final MechanismLigament2d wrist;
  private final String key;

  public ArmVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    MechanismRoot2d root = mechanism.getRoot("pivot",
        0.0,
        0.0);
    arm = new MechanismLigament2d("arm", ArmConstants.ARM_LENGTH_METERS, 0.0, 6, new Color8Bit(color));
    wrist = new MechanismLigament2d("wrist", ArmConstants.WRIST_LENGTH_METERS, 45.0, 5, new Color8Bit(color));
    root.append(arm);
    arm.append(wrist);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double armAngleDegs, double wristAngleDegs) {
    // Log Mechanism2d
    arm.setAngle(Rotation2d.fromDegrees(armAngleDegs));
    wrist.setAngle(Rotation2d.fromDegrees(wristAngleDegs));
    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivotArm =
            new Pose3d(ArmConstants.PIVOT_JOINT_TRANSLATION.getX(), 0.0,
                ArmConstants.PIVOT_JOINT_TRANSLATION.getY(),
                new Rotation3d(0.0, Units.degreesToRadians(armAngleDegs), 0.0));

    Pose3d pivotWrist = new Pose3d(AimbotUtils.getShooterTransformation(armAngleDegs).getTranslation(),
        new Rotation3d(0.0, Units.degreesToRadians(-wristAngleDegs), 0.0));
    Logger.recordOutput("Arm/Mechanism3d/" + key, pivotArm, pivotWrist);
  }
}
