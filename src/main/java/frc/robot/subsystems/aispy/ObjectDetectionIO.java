package frc.robot.subsystems.aispy;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ObjectDetectionIO {
  @AutoLog
  class ObjectDetectionIOInputs implements LoggableInputs {
    private boolean isConnected = false;
    private double timestamp = 0.0;
    private double[] observations = new double[] {};
    private long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", isConnected);
      table.put("Timestamp", timestamp);
      table.put("Observation Count", observations.length);
      for (int i = 0; i < observations.length; i++) {
        table.put("Observation/" + i, observations[0]);
      }
      table.put("FPS", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      isConnected = table.get("Connected", false);
      timestamp = table.get("Timestamp", timestamp);
      int observationCount = table.get("Observation Count", 0);
      observations = new double[observationCount];
      for (int i = 0; i < observationCount; i++) {
        observations[i] = table.get("Observation/" + i, observations[i]);
      }
      fps = table.get("FPS", fps);
    }

    public boolean isConnected() {
      return isConnected;
    }

    public void setConnected(boolean connected) {
      isConnected = connected;
    }

    public double getTimestamp() {
      return timestamp;
    }

    public void setTimestamp(double timestamp) {
      this.timestamp = timestamp;
    }

    public double[] getObservations() {
      return observations;
    }

    public void setObservations(double[] observations) {
      this.observations = observations;
    }

    public long getFps() {
      return fps;
    }

    public void setFps(long fps) {
      this.fps = fps;
    }
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
