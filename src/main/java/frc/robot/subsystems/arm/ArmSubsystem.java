package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs;
  private double m_desiredShoulderPose;
  private double m_desiredWristPose;

  public ArmSubsystem(ArmIO io) {
    m_io = io;
    m_inputs = new ArmIOInputsAutoLogged();

    m_desiredWristPose = -1;
    m_desiredShoulderPose = -1;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

    if (m_desiredWristPose != -1 && m_desiredShoulderPose != -1) {
      m_io.setShoulderAngle(m_desiredShoulderPose, false);

      // check to see if the wrist is currently too close to the rest of the arm
      if ((m_inputs.wristPositionRots * 360) - (360 - (m_inputs.shoulderPositionRots * 360)) < 15) {
        m_io.setWristAngle(m_inputs.wristPositionRots * 360, false);
      } else {
        m_io.setWristAngle(m_desiredWristPose, false);
      }
    }
  }

  public Command setArmDesiredPose(double shoulderPose, double wristPose) {
    return runOnce(() -> {
      m_desiredShoulderPose = shoulderPose;
      m_desiredWristPose = wristPose;
    });
  }

  public Command setShoulderPowerFactory(double power) {
    return runOnce(() -> m_io.setShoulderVoltage(power * 12.0));
  }

  public Command setShoulderPositionFactory(double degrees) {
    return runOnce(() -> m_desiredShoulderPose = degrees);
  }

  public Command setWristPowerFactory(double power) {
    return runOnce(() -> m_io.setWristVoltage(power * 12.0));
  }

  public Command setWristPositionFactory(double degrees) {
    return runOnce(() -> m_desiredWristPose = degrees);
  }

  public Command stopArmFactory() {
    return runOnce(() -> {
      m_io.setWristVoltage(0.0);
      m_io.setShoulderVoltage(0.0);
    });
  }
}

