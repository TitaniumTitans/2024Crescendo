package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import lib.utils.AimbotUtils;
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

  private ArmState m_desiredState = ArmState.DISABLED;
  private ArmState m_currentState = ArmState.DISABLED;

  private final ArmVisualizer m_setpointVisualizer;
  private final ArmVisualizer m_poseVisualizer;

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

    m_poseVisualizer = new ArmVisualizer("Current Arm Pose", Color.kFirstBlue);
    m_setpointVisualizer = new ArmVisualizer("Current Arm Setpoint", Color.kFirstRed);
  }

  //TODO: Finite state machine logic

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

    // clamp values for PID in between acceptable ranges
    m_desiredWristPoseDegs = m_desiredWristPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredWristPoseDegs, ArmConstants.WRIST_LOWER_LIMIT.getValue(),
            ArmConstants.WRIST_UPPER_LIMIT.getValue())
        : m_desiredWristPoseDegs;

    m_desiredArmPoseDegs = m_desiredArmPoseDegs > Double.NEGATIVE_INFINITY ?
        MathUtil.clamp(m_desiredArmPoseDegs, ArmConstants.ARM_LOWER_LIMIT.getValue(),
            ArmConstants.ARM_UPPER_LIMIT.getValue())
        : m_desiredArmPoseDegs;

    // check to make sure we're not in manual control
    m_io.enableBrakeMode(m_desiredArmPoseDegs <= Double.NEGATIVE_INFINITY
            || m_desiredWristPoseDegs <= Double.NEGATIVE_INFINITY);

    if (m_desiredArmPoseDegs > Double.NEGATIVE_INFINITY) {
      m_io.setArmAngle(m_desiredArmPoseDegs);
      Logger.recordOutput("Arm/Arm Setpoint Degs", m_desiredArmPoseDegs);
    }

    if (m_desiredWristPoseDegs > Double.NEGATIVE_INFINITY) {
      // check to see if the wrist is currently too close to the rest of the arm
      double wristGap = m_inputs.wristPositionDegs + m_inputs.armPositionDegs;
      if (wristGap < ArmConstants.WRIST_ARM_GAP.getValue()) {
        double underGap = ArmConstants.WRIST_ARM_GAP.getValue() - wristGap;
        m_io.setWristAngle(m_inputs.wristPositionDegs + underGap, true);
        Logger.recordOutput("Arm/Wrist Setpoint Degs", m_inputs.wristPositionDegs + underGap);
      } else {
        m_io.setWristAngle(m_desiredWristPoseDegs, false);
        Logger.recordOutput("Arm/Wrist Setpoint Degs", m_desiredWristPoseDegs);
      }
    }

    Logger.recordOutput("Arm/Wrist Gap", m_inputs.wristPositionDegs + m_inputs.armPositionDegs);

    // Update arm visualizers
    m_poseVisualizer.update(m_inputs.armPositionDegs, m_inputs.wristPositionDegs);
    m_setpointVisualizer.update(m_desiredArmPoseDegs, m_desiredWristPoseDegs);
  }

  public void handleState() {
    switch(m_currentState) {
      case STOW -> {
        m_desiredArmPoseDegs = ArmSetpoints.STOW_SETPOINT.armPoseDegs();
        m_desiredWristPoseDegs = ArmSetpoints.STOW_SETPOINT.wristPoseDegs();
      }
      case AUTO_AIM -> {
        ArmSetpoints.ArmSetpoint aimSetpoint = AimbotUtils.aimbotCalculate(
                new Pose3d(m_poseSupplier.get()), m_inputs.armPositionDegs);

        m_desiredArmPoseDegs = aimSetpoint.armPoseDegs();
        m_desiredWristPoseDegs = aimSetpoint.wristPoseDegs();
      }
      case INTAKE -> {
        m_desiredArmPoseDegs = ArmSetpoints.INTAKE_SETPOINT.armPoseDegs();
        m_desiredWristPoseDegs = ArmSetpoints.INTAKE_SETPOINT.wristPoseDegs();
      }
      default -> {
        m_desiredArmPoseDegs = m_inputs.armPositionDegs;
        m_desiredWristPoseDegs = m_inputs.wristPositionDegs;
      }
    }
  }



  /* Command Factories */

  public Command setDesiredState(ArmState state) {
    return runOnce(() -> m_desiredState = state);
  }

  public Command setArmDesiredPose(double armPose, double wristPose) {
    return run(() -> {
      m_desiredArmPoseDegs = armPose;

      double estWristGap = armPose + wristPose;
      if (estWristGap < ArmConstants.WRIST_ARM_GAP.getValue()) {
        double underGap = ArmConstants.WRIST_ARM_GAP.getValue() - estWristGap;
        m_desiredWristPoseDegs = wristPose + underGap;
      } else {
        m_desiredWristPoseDegs = wristPose;
      }
    });
  }

  public Command setArmPositionFactory(double degrees) {
    return run(() -> m_desiredArmPoseDegs = degrees);
  }

  public Command setWristPositionFactory(double degrees) {
    return run(() -> m_desiredWristPoseDegs = degrees);
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

