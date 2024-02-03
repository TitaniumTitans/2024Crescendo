package frc.robot.subsystems.aispy;

import edu.wpi.first.networktables.*;
import org.littletonrobotics.frc2023.util.Alert;

public class ObjectDetectionIOAISpy {
  private static final int CAMERA_ID = 0;
  private static final int CAMERA_RESOLUTION_WIDTH = 0;
  private static final int CAMERA_RESOLUTION_HEIGHT = 0;
  private static final int CAMERA_AUTO_EXPOSURE = 1;
  private static final int CAMERA_EXPOSURE = 10;
  private static final int MAX_TARGETS = 10;
  private static final double DETECTION_THRESHOLD = 0.75;

  private final DoubleArraySubscriber m_observationSubscriber;
  private final IntegerSubscriber m_fpsSubscriber;
  private final BooleanSubscriber m_connectionSubscriber;

  private final Alert m_disconnectAlert;

  public ObjectDetectionIOAISpy(String identifier) {
    var aispyTable = NetworkTableInstance.getDefault().getTable(identifier);

    var configTable = aispyTable.getSubTable("config");
    configTable.getIntegerTopic("camera_id").publish().set(CAMERA_ID);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(CAMERA_RESOLUTION_WIDTH);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(CAMERA_RESOLUTION_HEIGHT);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(CAMERA_AUTO_EXPOSURE);
    configTable.getIntegerTopic("camera_exposure").publish().set(CAMERA_EXPOSURE);
    configTable.getDoubleTopic("detection_threshold").publish().set(DETECTION_THRESHOLD);
    configTable.getIntegerTopic("max_targets").publish().set(MAX_TARGETS);

    var outputTable = aispyTable.getSubTable("output");
    m_observationSubscriber = outputTable.getDoubleArrayTopic("observations")
        .subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    m_fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
    m_connectionSubscriber = outputTable.getBooleanTopic("connected").subscribe(false);

    m_disconnectAlert = new Alert("Disconnected from AI-Spy instance " + identifier, Alert.AlertType.ERROR);
  }

  public void updateInputs(ObjectDetectionIO.ObjectDetectionIOInputs inputs) {
    inputs.setObservations(m_observationSubscriber.get());
    inputs.setFps(m_fpsSubscriber.get());
    inputs.setTimestamp(inputs.getObservations()[1]);
    inputs.setConnected(m_connectionSubscriber.get());

    m_disconnectAlert.set(!inputs.isConnected());
  }
}
