package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import lib.logger.DataLogUtil;

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
  private final ArmIO.ArmIOInputs m_inputs;
  private double m_desiredArmPoseDegs;
  private double m_armVelocityMult = 0;
  private double m_desiredWristPoseDegs;
  private double m_wristGap;
  private double m_wristVelocityMult = 0;
  private double m_startTime;

  private ArmState m_desiredState = ArmState.DISABLED;
  private ArmState m_currentState = ArmState.DISABLED;


  private final DataLogUtil.DataLogTable logUtil = DataLogUtil.getTable("Arm");

  private final ArmVisualizer m_setpointVisualizer;
  private final ArmVisualizer m_poseVisualizer;

  private final Supplier<Pose2d> m_poseSupplier;

  public ArmSubsystem(ArmIO io) {
    this(io, Pose2d::new);
  }

  public ArmSubsystem(ArmIO io, Supplier<Pose2d> supplier) {
    m_io = io;
    m_inputs = new ArmIO.ArmIOInputs();

    m_desiredWristPoseDegs = Double.NEGATIVE_INFINITY;
    m_desiredArmPoseDegs = Double.NEGATIVE_INFINITY;

    m_io.resetPosition();

    m_poseSupplier = supplier;

    setupLogging();
    m_poseVisualizer = new ArmVisualizer("Current Arm Pose", Color.kFirstBlue);
    m_setpointVisualizer = new ArmVisualizer("Current Arm Setpoint", Color.kFirstRed);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    handleState();

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
      m_io.setWristAngle(m_desiredWristPoseDegs, m_wristVelocityMult);

      double underGap = MathUtil.clamp(ArmConstants.WRIST_ARM_GAP.getValue()
          - (m_inputs.armPositionDegs + m_inputs.wristPositionDegs), 0, 180);

      // set the arms angle
      m_io.setArmAngle(m_desiredArmPoseDegs, m_armVelocityMult);
    }

    // Update arm visualizers
    m_poseVisualizer.update(m_inputs.armPositionDegs, m_inputs.wristPositionDegs);
    m_setpointVisualizer.update(m_desiredArmPoseDegs, m_desiredWristPoseDegs);
  }

  public void handleState() {
    switch(m_desiredState) {
      case STOW -> {
        if (m_inputs.armPositionDegs > 60 && m_currentState == ArmState.AMP) {
          m_wristVelocityMult = 0.0;
          m_armVelocityMult = 1.0;
        } else {
          m_currentState = ArmState.STOW;
          m_armVelocityMult = 1.0;
          m_wristVelocityMult = 1.0;
        }

        m_desiredArmPoseDegs = ArmSetpoints.STOW_SETPOINT.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.STOW_SETPOINT.wristAngle();
      }
      case AUTO_AIM -> {
        m_armVelocityMult = 1.0;
        m_wristVelocityMult = 1.0;

        m_desiredArmPoseDegs = ArmConstants.WRIST_ARM_GAP.getValue() - m_desiredWristPoseDegs;
        m_desiredArmPoseDegs = m_desiredArmPoseDegs >= ArmConstants.ARM_LOWER_LIMIT.getValue() ? m_desiredArmPoseDegs
            : ArmConstants.ARM_LOWER_LIMIT.getValue();
        m_desiredWristPoseDegs = ArmSetpoints.WRIST_ANGLE.getValue();
      }
      case INTAKE -> {
        m_armVelocityMult = 1.0;
        m_wristVelocityMult = 1.0;

        m_currentState = ArmState.INTAKE;
        m_desiredArmPoseDegs = ArmSetpoints.INTAKE_SETPOINT.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.INTAKE_SETPOINT.wristAngle();
      }
      case AMP -> {
        if (Math.abs(m_inputs.wristPositionDegs - m_desiredWristPoseDegs) > 5) {
          m_armVelocityMult = 0.5;
          m_wristVelocityMult = 1.0;
        } else {
          m_armVelocityMult = 1.0;
          m_wristVelocityMult = 1.0;
        }

        m_currentState = ArmState.AMP;

        m_desiredArmPoseDegs = ArmSetpoints.AMP_SETPOINT.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.AMP_SETPOINT.wristAngle();
      }
      default -> {
        m_armVelocityMult = 1.0;
        m_wristVelocityMult = 1.0;

        m_desiredArmPoseDegs = m_inputs.armPositionDegs;
        m_desiredWristPoseDegs = m_inputs.wristPositionDegs;
      }
    }
  }

  public void setDesiredState(ArmState state) {
    m_desiredState = state;
  }

  public boolean armAtSetpoint() {
    return Math.abs(m_inputs.armPositionDegs - m_desiredArmPoseDegs) < 5.0;
  }

  public boolean wristAtSetpoint() {
    return Math.abs(m_inputs.wristPositionDegs - m_desiredWristPoseDegs) < 5.0;
  }

  public boolean bothAtSetpoint() {
    return armAtSetpoint() && wristAtSetpoint();
  }

  /* Command Factories */

  public Command setDesiredStateFactory(ArmState state) {
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

  /** Logging util */
  public void setupLogging() {
    logUtil.addDouble("ArmAngleDegs", () -> m_inputs.armPositionDegs, true);
    logUtil.addDouble("WristAngleDegs", () -> m_inputs.wristPositionDegs, true);

    logUtil.addDouble("ArmSetpointDegs", () -> m_desiredArmPoseDegs, true);
    logUtil.addDouble("WristSetpointDegs", () -> m_desiredWristPoseDegs, true);

    logUtil.addDouble("ArmAppliedOutput", () -> m_inputs.armAppliedOutput, false);
    logUtil.addDouble("WristAppliedOutput", () -> m_inputs.wristAppliedOutput, false);

    logUtil.addDouble("ArmClosedLoopOutput", () -> m_inputs.armClosedLoopOutput, false);
    logUtil.addDouble("WristClosedLoopOutput", () -> m_inputs.wristClosedLoopOutput, false);

    logUtil.addDouble("ArmAppliedOutput", () -> m_inputs.armAppliedOutput, false);
    logUtil.addDouble("WristAppliedOutput", () -> m_inputs.wristAppliedOutput, false);

    logUtil.addDouble("WristArmGap", () -> m_wristGap, true);
    logUtil.addString("Arm Current State", () -> m_desiredState.toString(), true);
  }
}

