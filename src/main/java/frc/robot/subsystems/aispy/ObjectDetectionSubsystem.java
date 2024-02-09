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

    for (int i = 1; i < obs[0]; i += 4) {
      m_observations.add(new DetectionObservation(
          obs[i],
          obs[i + 1],
          obs[i + 2],
          obs[i + 3]
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
      double deltaX,
      double deltaY,
      double confidence,
      double percentage
  ) {}
}

