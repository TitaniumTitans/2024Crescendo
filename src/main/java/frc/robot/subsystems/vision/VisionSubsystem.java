package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;

public class VisionSubsystem extends SubsystemBase {
     private final PhotonCamera m_camera;
     private final Transform3d robotToCam;
     private PhotonPoseEstimator m_photonPoseEstimator;
     private final DriverStation.Alliance m_alliance;
     private AprilTagFieldLayout m_aprilTagFieldLayout;
    public VisionSubsystem(String camName, Transform3d camPose) {
        m_camera = new PhotonCamera(camName);
        robotToCam = camPose;
        if(DriverStation.getAlliance()!=null) {
            m_alliance = DriverStation.getAlliance().get();
        } else {
            m_alliance = DriverStation.Alliance.Blue;
        }
        try {
            m_aprilTagFieldLayout =AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            if(m_alliance == DriverStation.Alliance.Blue){
                m_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            } else {
                m_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            }
            m_photonPoseEstimator = new PhotonPoseEstimator(
                    m_aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    m_camera,
                    robotToCam);
        } catch (IOException e){
            throw new IllegalStateException(e);
        }

    }
}

