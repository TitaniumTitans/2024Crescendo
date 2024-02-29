package frc.robot.subsystems.arm;

import com.gos.lib.properties.GosDoubleProperty;

public class ArmPose {
  private final GosDoubleProperty m_wristAngleProperty;
  private final GosDoubleProperty m_armAngleProperty;

  private final double m_armAngle;
  private final double m_wristAngle;

  public ArmPose(String name, boolean isConstant, double armAngle, double wristAngle) {
    m_armAngleProperty = new GosDoubleProperty(isConstant, name + "Arm Angle", armAngle);
    m_wristAngleProperty = new GosDoubleProperty(isConstant, name + "Wrist Angle", wristAngle);

    m_armAngle = 0.0;
    m_wristAngle = 0.0;
  }

  public ArmPose(double armAngle, double wristAngle) {
    m_armAngleProperty = null;
    m_wristAngleProperty = null;

    m_armAngle = armAngle;
    m_wristAngle = wristAngle;
  }

  public double armAngle() {
    if (m_armAngleProperty != null) {
      return m_armAngleProperty.getValue();
    }
    return m_armAngle;
  }

  public double wristAngle() {
    if (m_wristAngleProperty != null) {
      return m_wristAngleProperty.getValue();
    }

    return m_wristAngle;
  }
}
