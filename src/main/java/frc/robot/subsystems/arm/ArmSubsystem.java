package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.utils.AllianceFlipUtil;
import lib.utils.FieldConstants;
import lib.utils.GeomUtils;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

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
  public enum ArmStates {
    STOW,
    AIM,
    AIM_BLOCKED,
    AMP,
    TRAP,
    MOVING
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

  public Transform3d getShooterTransformation() {
    return new Transform3d(new Pose3d(),
        Constants.ArmConstants.PIVOT_TRANSLATION_METERS.plus(
            GeomUtils.translationToTransform(new Translation3d(
                Constants.ArmConstants.SHOULDER_BAR_LENGTH_METERS,
                new Rotation3d(0.0, m_inputs.shoulderPositionRots * Math.PI * 2, 0.0)
            ))
        ));
  }

  public double calcShooterAngle(Pose3d robotPose) {
    Pose3d speakerPose = new Pose3d(AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER), new Rotation3d());
    Pose3d shooterPivotPose = robotPose.plus(getShooterTransformation());
    Transform3d robotToSpeaker = new Transform3d(shooterPivotPose.plus(getShooterTransformation()), speakerPose);

    double groundDistance =
        Math.sqrt(Math.pow(robotToSpeaker.getX(), 2) + Math.pow(robotToSpeaker.getY(), 2));
    return Math.atan2(groundDistance, robotToSpeaker.getZ());
  }
}

