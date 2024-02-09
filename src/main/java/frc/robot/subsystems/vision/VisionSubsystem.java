package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PoseEstimator;
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

  private final PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();

  public void updateInputs(){
    inputs.setCamConnected(m_camera.isConnected());
    inputs.setCamLatency(m_camera.getLatestResult().getLatencyMillis());
    inputs.setTagIds(getTargetIDs());
    inputs.setNumTagsFound(m_camera.getLatestResult().targets.size());
    inputs.setTagsFound(m_camera.getLatestResult().hasTargets());

    Logger.processInputs("PhotonVision/" + m_name, inputs);
  }

  public VisionSubsystem(String camName, Transform3d camPose) {
    m_camera = new PhotonCamera(camName);
    m_name = camName;
    robotToCam = camPose;
    try {
      m_aprilTagFieldLayout =AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      m_photonPoseEstimator = new PhotonPoseEstimator(
          m_aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          m_camera,
          robotToCam);
    } catch (IOException e){
      throw new IllegalStateException(e);
    }
  }

  public Optional<PoseEstimator.TimestampedVisionUpdate> getPose(Pose2d prevEstimatedRobotPose) {

    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    PhotonPipelineResult camResult = m_camera.getLatestResult();
    Optional<EstimatedRobotPose> opPose = m_photonPoseEstimator.update(camResult);

    if (opPose.isEmpty()) {
      return Optional.empty();
    } else {
      EstimatedRobotPose estPose = opPose.get();
      return Optional.of(new PoseEstimator.TimestampedVisionUpdate(estPose.timestampSeconds,
          estPose.estimatedPose.toPose2d(),
          VecBuilder.fill(Units.inchesToMeters(0.5), Units.inchesToMeters(0.5), Units.degreesToRadians(15))));
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

  public record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
    public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {
      // Apply vision updates
      // Calculate Kalman gains based on std devs
      // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
      Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
      var r = new double[3];
      for (int i = 0; i < 3; ++i) {
        r[i] = this.stdDevs().get(i, 0) * this.stdDevs().get(i, 0);
      }
      for (int row = 0; row < 3; ++row) {
        if (q.get(row, 0) == 0.0) {
          visionK.set(row, row, 0.0);
        } else {
          visionK.set(
              row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
        }
      }

      // Calculate twist between current and vision pose
      var visionTwist = lastPose.log(this.pose());

      // Multiply by Kalman gain matrix
      var twistMatrix =
          visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));

      // Apply twist
      lastPose =
          lastPose.exp(
              new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));

      return lastPose;
    }
  }
}

