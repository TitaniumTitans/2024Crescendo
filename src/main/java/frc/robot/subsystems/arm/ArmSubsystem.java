package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    STOW,
    INTAKE,
    AUTO_AIM,
    ANTI_DEFENSE,
    AMP,
    TRANSITION_AMP,
    SOURCE,
    TRANSITION_SOURCE,
    DISABLED
  }

  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs;
  private double m_desiredArmPoseDegs;
  private double m_desiredWristPoseDegs;
  private double m_startTime;

  private ArmState m_desiredState = ArmState.DISABLED;
  private ArmState m_currentState = ArmState.DISABLED;

//  private final ArmVisualizer m_setpointVisualizer;
//  private final ArmVisualizer m_poseVisualizer;

  private final Supplier<Pose2d> m_poseSupplier;

  public ArmSubsystem(ArmIO io) {
    this(io, Pose2d::new);
  }

  public ArmSubsystem(ArmIO io, Supplier<Pose2d> supplier) {
    m_io = io;
    m_inputs = new ArmIOInputsAutoLogged();

    m_desiredWristPoseDegs = Double.NEGATIVE_INFINITY;
    m_desiredArmPoseDegs = Double.NEGATIVE_INFINITY;

    m_io.resetPosition();

    m_poseSupplier = supplier;

//    m_poseVisualizer = new ArmVisualizer("Current Arm Pose", Color.kFirstBlue);
//    m_setpointVisualizer = new ArmVisualizer("Current Arm Setpoint", Color.kFirstRed);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

    handleState();
//    Logger.recordOutput("Arm/Arm Desired State", m_desiredState.toString());

    // clamp values for PID in between acceptable ranges
    m_desiredWristPoseDegs = m_desiredWristPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredWristPoseDegs, ArmConstants.WRIST_LOWER_LIMIT.getValue(),
            ArmConstants.WRIST_UPPER_LIMIT.getValue())
        : m_desiredWristPoseDegs;

    m_desiredArmPoseDegs = m_desiredArmPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredArmPoseDegs, ArmConstants.ARM_LOWER_LIMIT.getValue(),
            ArmConstants.ARM_UPPER_LIMIT.getValue())
        : m_desiredArmPoseDegs;

    // if we're disabled go back to hold pose
    if (DriverStation.isDisabled()) {
      m_desiredState = ArmState.DISABLED;
    }

    // check to make sure we're not in manual control
    m_io.enableBrakeMode(m_desiredState == ArmState.DISABLED);

    if (m_desiredState != ArmState.DISABLED) {
      // check to see if the wrist is currently too close to the rest of the arm
      m_io.setWristAngle(m_desiredWristPoseDegs, false);
      Logger.recordOutput("Arm/Wrist Setpoint Degs", m_desiredWristPoseDegs);

      // set the arms angle
      m_io.setArmAngle(m_desiredArmPoseDegs);
      Logger.recordOutput("Arm/Arm Setpoint Degs", m_desiredArmPoseDegs);
    }
  }

  public void handleState() {
    switch(m_desiredState) {
      case STOW -> {
        if (m_currentState == ArmState.AMP
            && m_inputs.armPositionDegs < ArmSetpoints.AMP_INTERMEDIATE.armAngle() + 5
            && m_inputs.wristPositionDegs < ArmSetpoints.AMP_INTERMEDIATE.wristAngle() + 5) {
          m_currentState = ArmState.STOW;
        } else if (m_currentState == ArmState.AMP) {
          m_desiredArmPoseDegs = ArmSetpoints.AMP_INTERMEDIATE.armAngle();
          m_desiredWristPoseDegs = ArmSetpoints.AMP_INTERMEDIATE.wristAngle();
        } else {
          m_desiredArmPoseDegs = ArmSetpoints.STOW_SETPOINT.armAngle();
          m_desiredWristPoseDegs = ArmSetpoints.STOW_SETPOINT.wristAngle();
        }
      }
      case AUTO_AIM -> {
        m_desiredArmPoseDegs = ArmConstants.WRIST_ARM_GAP.getValue() - m_desiredWristPoseDegs;
        m_desiredArmPoseDegs = m_desiredArmPoseDegs >= ArmConstants.ARM_LOWER_LIMIT.getValue() ? m_desiredArmPoseDegs
            : ArmConstants.ARM_LOWER_LIMIT.getValue();
        m_desiredWristPoseDegs = ArmSetpoints.WRIST_ANGLE.getValue();
      }
      case INTAKE -> {
        m_currentState = ArmState.INTAKE;
        m_desiredArmPoseDegs = ArmSetpoints.INTAKE_SETPOINT.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.INTAKE_SETPOINT.wristAngle();
      }
      case AMP -> {
        if (m_currentState != ArmState.TRANSITION_AMP && m_currentState != ArmState.AMP) {
          m_startTime = Timer.getFPGATimestamp();
          m_currentState = ArmState.TRANSITION_AMP;
        }

        if (m_currentState == ArmState.TRANSITION_AMP) {
          double trajTime = Timer.getFPGATimestamp() - m_startTime;
          Trajectory.State armState = ArmSetpoints.STOW_AMP_TRAJ.sample(trajTime);
        }
      }
      default -> {
        m_desiredArmPoseDegs = m_inputs.armPositionDegs;
        m_desiredWristPoseDegs = m_inputs.wristPositionDegs;
      }
    }
  }

  /* Command Factories */

  public Command setDesiredState(ArmState state) {
    return runEnd(() -> m_desiredState = state,
        () -> m_desiredState = ArmState.STOW);
  }

  public Command enableBrakeMode(boolean enabled) {
    return runOnce(() -> m_io.enableBrakeMode(enabled)).ignoringDisable(true);
  }

  public Command resetEncoderFactory() {
    return runOnce(m_io::resetPosition).ignoringDisable(true);
  }

  public Command setArmPowerFactory(double power) {
    return runEnd(() -> {
          // let arm know it's in manual control
          m_io.enableBrakeMode(true);
          m_desiredArmPoseDegs = Double.NEGATIVE_INFINITY;

          double finalPower = power;
          // limiting code for arm
          if (m_inputs.armPositionDegs > ArmConstants.ARM_UPPER_LIMIT.getValue()) {
            finalPower = MathUtil.clamp(power, -1.0, 0.0);
          } else if (m_inputs.armPositionDegs < ArmConstants.ARM_LOWER_LIMIT.getValue()) {
            finalPower = MathUtil.clamp(power, 0.0, 1.0);
          }
          m_io.setArmVoltage(finalPower * 12.0);

          // check to see if wrist is too close, if it is back drive it
          if (m_inputs.armPositionDegs + m_inputs.wristPositionDegs < ArmConstants.WRIST_ARM_GAP.getValue()) {
            m_desiredWristPoseDegs = Double.NEGATIVE_INFINITY;
            m_io.setWristVoltage(Math.abs(finalPower));
          }
        },
        () -> m_io.setArmVoltage(0.0));
  }

  public Command setWristPowerFactory(double power) {
    return runEnd(() -> {
          // let wrist know it's in manual control mode
          m_io.enableBrakeMode(true);
          m_desiredWristPoseDegs = Double.NEGATIVE_INFINITY;

          // limiting code for wrist
          final double outPower;
          if (m_inputs.wristPositionDegs > ArmConstants.WRIST_UPPER_LIMIT.getValue()) {
            outPower = MathUtil.clamp(power, -1.0, 0.0);
          } else if (m_inputs.wristPositionDegs < ArmConstants.WRIST_LOWER_LIMIT.getValue()
              || m_inputs.wristPositionDegs + m_inputs.armPositionDegs < ArmConstants.WRIST_ARM_GAP.getValue()) {
            outPower = MathUtil.clamp(power, 0.0, 1.0);
          } else {
            outPower = power;
          }
          m_io.setWristVoltage(outPower * 12.0);
        },
        () -> m_io.setWristVoltage(0.0));
  }


}

