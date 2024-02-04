package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.utils.FieldConstants;
import lib.utils.AllianceFlipUtil;

public class Supersystem extends SubsystemBase {
  public Supersystem() {

  }

  public double calcShooterAngle(Pose2d robotPose) {
    Pose3d speakerPose = new Pose3d(AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER), new Rotation3d());
    double groundDistance = Math.sqrt(Math.pow(speakerPose.getX(), 2) + Math.pow(speakerPose.getY(), 2));
    return Math.atan2(groundDistance, FieldConstants.CENTER_SPEAKER.getZ());
  }

  public double calcMaxShooterAngle(Pose2d robotPose, double shoulderAngle) {
    return 0;
  }
}

