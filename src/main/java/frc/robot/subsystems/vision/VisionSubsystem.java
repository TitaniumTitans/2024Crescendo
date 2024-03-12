package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import lib.logger.DataLogUtil;
import lib.utils.FieldConstants;
import lib.utils.PoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.Optional;

public class VisionSubsystem {

  private final PhotonCamera m_camera;
  private final Transform3d robotToCam;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  private final String m_name;

  private final double xyStdDevCoefficient = Units.inchesToMeters(4.0);
  private final double thetaStdDevCoefficient = Units.degreesToRadians(12.0);

  private final double xyStdDevMultiTagCoefficient = Units.inchesToMeters(2.0);
  private final double thetaStdDevMultiTagCoefficient = Units.degreesToRadians(6.0);

  private final PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();

  public VisionSubsystem(String camName, Transform3d camPose) {
    m_camera = new PhotonCamera(camName);
    m_name = camName;
    robotToCam = camPose;
    try {
      m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      m_photonPoseEstimator = new PhotonPoseEstimator(
          FieldConstants.APRIL_TAG_FIELD_LAYOUT,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          m_camera,
          robotToCam);

      m_photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    } catch (IOException e){
      throw new IllegalStateException(e);
    }
  }

  public void updateInputs() {
    inputs.setCamConnected(m_camera.isConnected());
    inputs.setCamLatency(m_camera.getLatestResult().getLatencyMillis());
    inputs.setTagIds(getTargetIDs());
    inputs.setNumTagsFound(m_camera.getLatestResult().targets.size());
    inputs.setTagsFound(m_camera.getLatestResult().hasTargets());

    Logger.processInputs("Vision/" + m_name, inputs);
  }

  public Optional<PoseEstimator.TimestampedVisionUpdate> getPose(Pose2d prevEstimatedRobotPose) {
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    PhotonPipelineResult camResult = m_camera.getLatestResult();

    Optional<EstimatedRobotPose> opPose = m_photonPoseEstimator.update(camResult);

    if (opPose.isEmpty()) {
      return Optional.empty();
    } else {
      EstimatedRobotPose estPose = opPose.get();

      // find average distance to tags
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : camResult.targets) {
        var tagPose = m_photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

        if (tagPose.isEmpty()) {
          continue;
        }

        numTags++;
        avgDist +=
            tagPose.get().toPose2d().getTranslation().getDistance(estPose.estimatedPose.toPose2d().getTranslation());
      }
      if (numTags == 0.0) {
        avgDist = 0.0;
      } else {
        avgDist /= numTags;
      }

      double xyStdDev = xyStdDevCoefficient * Math.pow(avgDist, 2.0);
      double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDist, 2.0);

      if (camResult.getTargets().size() > 1.0) {
        xyStdDev = xyStdDevMultiTagCoefficient * Math.pow(avgDist, 2.0);
        thetaStdDev = thetaStdDevMultiTagCoefficient * Math.pow(avgDist, 2.0);
      }

      Logger.recordOutput("Vision/" + m_name + "/Estimated Pose", estPose.estimatedPose);

      return Optional.of(new PoseEstimator.TimestampedVisionUpdate(estPose.timestampSeconds,
          estPose.estimatedPose.toPose2d(),
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
    }
  }

  public int[] getTargetIDs () {
    PhotonPipelineResult targets = m_camera.getLatestResult();
    int[] results;

    if(targets.hasTargets()) {
      results = new int[targets.getTargets().size()];

      for (int i = 0; i < results.length; i++) {
        results[i] = targets.getTargets().get(i).getFiducialId();
      }

    }
    else {
      results = new int[0];
    }

    return results;
  }
}

