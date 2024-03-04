package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO{
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(2), ArmConstants.ARM_SENSOR_MECHANISM_RATIO, 0.0060620304,
          ArmConstants.ARM_LENGTH_METERS, 0, Units.degreesToRadians(180), true, Units.degreesToRadians(45));

  private final SingleJointedArmSim m_wristSim = new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(2), ArmConstants.WRIST_SENSOR_MECHANISM_RATIO, 0.0060620304,
          ArmConstants.WRIST_LENGTH_METERS, 0.0, Units.degreesToRadians(180), true, Units.degreesToRadians(45));

  double m_armAppliedOutput = 0.0;
  double m_wristAppliedOutput = 0.0;

  private final PIDController m_armController =
          new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
  private final PIDController m_wristController =
          new PIDController(ArmConstants.WRIST_KP, ArmConstants.WRIST_KI, ArmConstants.WRIST_KD);

  private static final double LOOP_PERIOD_SECS = 0.02;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    m_armSim.update(LOOP_PERIOD_SECS);
    m_wristSim.update(LOOP_PERIOD_SECS);

    inputs.armPositionDegs = Units.radiansToDegrees(m_armSim.getAngleRads());
    inputs.wristPositionDegs = Units.radiansToDegrees(m_wristSim.getAngleRads());

    inputs.armVelocityDegsPerSecond = Units.radiansToDegrees(-m_armSim.getVelocityRadPerSec());
    inputs.wristVelocityDegsPerSecond = Units.radiansToDegrees(m_wristSim.getVelocityRadPerSec());
  }

  @Override
  public void setArmVoltage(double voltage) {
    m_armAppliedOutput = MathUtil.clamp(voltage, -12.0, 12.0);
    m_armSim.setInputVoltage(m_armAppliedOutput);
  }

  @Override
  public void setArmAngle(double degrees) {
    double setpointRots = Units.degreesToRotations(degrees);
    m_armAppliedOutput = m_armController.calculate(Units.radiansToRotations(m_armSim.getAngleRads()), setpointRots);
    m_armAppliedOutput = MathUtil.clamp(m_armAppliedOutput, -12.0, 12.0);
    m_armSim.setInputVoltage(m_armAppliedOutput);
  }

  @Override
  public void setWristVoltage(double voltage) {
    m_wristAppliedOutput = MathUtil.clamp(voltage, -12.0, 12.0);
    m_wristSim.setInputVoltage(m_wristAppliedOutput);
  }

  @Override
  public void setWristAngle(double degrees, boolean track) {
    double setpointRots = Units.degreesToRotations(degrees);
    m_wristAppliedOutput = m_wristController.calculate(Units.radiansToRotations(m_wristSim.getAngleRads()), setpointRots);
    m_wristAppliedOutput = MathUtil.clamp(m_wristAppliedOutput, -12.0, 12.0);
    m_wristSim.setInputVoltage(m_wristAppliedOutput);
  }

  @Override
  public void resetPosition() {
    m_armSim.setState(0.0, 0.0);
    m_wristSim.setState(Units.degreesToRadians(45.0), 0.0);
  }
}
