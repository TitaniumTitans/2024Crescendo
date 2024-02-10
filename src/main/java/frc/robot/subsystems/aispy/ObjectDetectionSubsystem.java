package frc.robot.subsystems.aispy;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private final ObjectDetectionIO m_io;
  private final ObjectDetectionIO.ObjectDetectionIOInputs m_inputs;
  private ArrayList<DetectionObservation> m_observations;
  public ObjectDetectionSubsystem(ObjectDetectionIO io) {
    m_io = io;
    m_inputs = new ObjectDetectionIO.ObjectDetectionIOInputs();
    m_observations = new ArrayList<>();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_observations.clear();

    double[] obs = m_inputs.getObservations();

    for (int i = 2; i < obs[0]; i += 5) {
      m_observations.add(new DetectionObservation(
          (int) obs[i], // Object ID
          obs[i + 1], // Object tX
          obs[i + 2], // Object tY
          obs[i + 3], // Object confidence
          obs[i + 4] // Object area
      ));
    }
  }

  public DetectionObservation getBestDetection() {
    m_observations.sort(
        Comparator.comparingDouble((DetectionObservation det) -> det.confidence));
    return m_observations.get(0);
  }

  public DetectionObservation getLargestDetection() {
    m_observations.sort(
        Comparator.comparingDouble((DetectionObservation det) -> det.percentage));
    return m_observations.get(0);
  }

  public record DetectionObservation(
      int id,
      double deltaX,
      double deltaY,
      double confidence,
      double percentage
  ) {}
}

