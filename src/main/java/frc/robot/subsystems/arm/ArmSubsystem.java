package frc.robot.subsystems.arm;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs;
  private double m_desiredArmPoseDegs;
  private double m_desiredWristPoseDegs;
  private static final boolean USE_MM = true;

  public ArmSubsystem(ArmIO io) {
    m_io = io;
    m_inputs = new ArmIOInputsAutoLogged();

    m_desiredWristPoseDegs = Double.NEGATIVE_INFINITY;
    m_desiredArmPoseDegs = Double.NEGATIVE_INFINITY;

    m_io.resetPosition();
  }

  //TODO: Finite state machine logic

  public double getWristPosition() {
    return m_inputs.wristPositionDegs;
  }

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
    if (m_desiredArmPoseDegs > Double.NEGATIVE_INFINITY && m_desiredWristPoseDegs > Double.NEGATIVE_INFINITY) {
      m_io.enableBrakeMode(false);
    } else {
      m_io.enableBrakeMode(true);
    }

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
  }

  public Command setArmDesiredPose(double armPose, double wristPose) {
    return run(() -> {
      m_desiredArmPoseDegs = armPose;

      double wristGap = m_inputs.wristPositionDegs + m_inputs.armPositionDegs;
      if (wristGap < ArmConstants.WRIST_ARM_GAP.getValue()) {
        double underGap = ArmConstants.WRIST_ARM_GAP.getValue() - wristGap;
        m_desiredWristPoseDegs = m_inputs.wristPositionDegs + underGap;
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

