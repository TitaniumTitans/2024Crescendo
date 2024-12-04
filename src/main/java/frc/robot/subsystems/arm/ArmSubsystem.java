package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import lib.utils.AimbotUtils;
import lib.utils.ArmTrajectory;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    STOW,
    INTAKE,
    AUTO_AIM,
    ANTI_DEFENSE,
    AMP,
    AMP_REVERSE,
    PREPARE_TRAP,
    SCORE_TRAP,
    PASS,
    DISABLED,
    BACKUP_SHOT,
    MANUAL_CONTROL,
  }

  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs;
  private double m_desiredArmPoseDegs;
  private double m_armVelocityMult = 0;
  private double m_desiredWristPoseDegs;
  private double m_wristVelocityMult = 0;
  private boolean m_disabledBrakeMode = true;

  private final Timer m_trajTimer;
  private double m_reverseTimer;
  private double m_forwardTimer;
  private ArmTrajectory.ArmTrajectoryState armState;

  private ArmState m_desiredState = ArmState.STOW;
  private ArmState m_currentState = ArmState.DISABLED;

  private double m_wristIncremental = 45.0;
  private double m_armIncremental = 0.0;

  private final ArmVisualizer m_setpointVisualizer;
  private final ArmVisualizer m_poseVisualizer;

  private final Supplier<Pose2d> m_poseSupplier;
  private final BooleanSupplier m_climberLock;
  private final DoubleSupplier m_climberHeightSupplier;

  public ArmSubsystem(ArmIO io) {
    this(io, Pose2d::new, () -> false, () -> 0.0);
  }

  public ArmSubsystem(ArmIO io, Supplier<Pose2d> supplier, BooleanSupplier climberLock, DoubleSupplier climberHeight) {
    m_io = io;
    m_inputs = new ArmIOInputsAutoLogged();

    m_desiredWristPoseDegs = Double.NEGATIVE_INFINITY;
    m_desiredArmPoseDegs = Double.NEGATIVE_INFINITY;

    m_io.resetPosition();

    m_poseSupplier = supplier;
    m_climberLock = climberLock;
    m_climberHeightSupplier = climberHeight;

    m_trajTimer = new Timer();
    m_forwardTimer = 0.0;
    m_reverseTimer = 0.0;
    armState = null;

    m_poseVisualizer = new ArmVisualizer("Current Arm Pose", Color.kFirstBlue);
    m_setpointVisualizer = new ArmVisualizer("Current Arm Setpoint", Color.kFirstRed);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

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
    m_io.enableBrakeMode(m_desiredState == ArmState.DISABLED && m_disabledBrakeMode);

    boolean useMM = armState != null;

    if (m_desiredState != ArmState.DISABLED) {
      // check to see if the wrist is currently too close to the rest of the arm
      double predictedUnderGap = MathUtil.clamp(ArmConstants.WRIST_ARM_GAP.getValue()
              - (m_desiredArmPoseDegs + m_desiredWristPoseDegs), 0, 180);

      m_io.setWristAngle(m_desiredWristPoseDegs + predictedUnderGap, m_wristVelocityMult, useMM);

      // set the arms angle
      m_io.setArmAngle(m_desiredArmPoseDegs, m_armVelocityMult, useMM);
    }

    Logger.recordOutput("Arm/Desired State", m_desiredState);
    Logger.recordOutput("Arm/Current State", m_currentState);

    Logger.recordOutput("Arm/Arm Setpoint", m_desiredArmPoseDegs);
    Logger.recordOutput("Arm/Wrist Setpoint", m_desiredWristPoseDegs);

    Logger.recordOutput("Arm/At Setpoint", armAtSetpoint());
    Logger.recordOutput("Arm/Traj Timer", m_trajTimer.get());

    Logger.recordOutput("Arm/Reverse Timer", m_reverseTimer);
    Logger.recordOutput("Arm/Forward Timer", m_forwardTimer);

//    Logger.recordOutput("Arm/Arm Velocity Multiplier");
//    Logger.recordOutput("Arm/Wrist Velocity Multiplier");

    // Update arm visualizers
    m_poseVisualizer.update(m_inputs.armPositionDegs, m_inputs.wristPositionDegs);
    m_setpointVisualizer.update(m_desiredArmPoseDegs, m_desiredWristPoseDegs);
  }

  public void handleState() {
    // handle climber locks
    if (m_climberLock.getAsBoolean()
    && m_currentState != ArmState.PREPARE_TRAP
    && m_currentState != ArmState.SCORE_TRAP) {
      m_desiredState = ArmState.STOW;
    } else if (m_climberLock.getAsBoolean()
        && m_desiredState != ArmState.PREPARE_TRAP
        && m_desiredState != ArmState.SCORE_TRAP) {
      m_desiredState = ArmState.PREPARE_TRAP;
    }

    if (m_currentState == ArmState.SCORE_TRAP &&
        (m_desiredState != ArmState.SCORE_TRAP && m_desiredState != ArmState.PREPARE_TRAP)) {
      m_desiredState = ArmState.PREPARE_TRAP;
    }

    if (m_currentState == ArmState.SCORE_TRAP
      && m_climberHeightSupplier.getAsDouble() > 50.0) {
      m_desiredState = ArmState.SCORE_TRAP;
    }

    if (m_currentState == ArmState.PREPARE_TRAP
        && m_climberHeightSupplier.getAsDouble() > 50.0) {
      m_desiredState = ArmState.PREPARE_TRAP;
    }

    switch(m_desiredState) {
      case STOW -> {
        if (m_inputs.armPositionDegs > 60.0 &&
            (m_currentState == ArmState.AMP || m_currentState == ArmState.PREPARE_TRAP)) {
          m_wristVelocityMult = 0.15;
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

        double groundDistance = Units.metersToInches(AimbotUtils.getDistanceFromSpeaker(m_poseSupplier.get()));

        m_desiredWristPoseDegs = AimbotUtils.getWristAngle(groundDistance);

        m_desiredArmPoseDegs = ArmConstants.WRIST_ARM_GAP.getValue() - m_desiredWristPoseDegs;
        m_desiredArmPoseDegs = m_desiredArmPoseDegs >= 0 ? m_desiredArmPoseDegs : 0;

        m_currentState = ArmState.AUTO_AIM;
      }
      case ANTI_DEFENSE -> {
        m_desiredArmPoseDegs = 68.0;
        m_desiredWristPoseDegs = 65.0;
        m_currentState = ArmState.ANTI_DEFENSE;
      }
      case INTAKE -> {
        m_armVelocityMult = 1.0;
        m_wristVelocityMult = 1.0;

        m_currentState = ArmState.INTAKE;
        m_desiredArmPoseDegs = ArmSetpoints.INTAKE_SETPOINT.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.INTAKE_SETPOINT.wristAngle();
      }
      case AMP -> {
        ArmTrajectory traj = ArmConstants.AMP_TRAJECTORY;

        // arm state should be null by the time the trajectory ends
        if (m_currentState != ArmState.AMP) {
          if (m_currentState != ArmState.AMP_REVERSE) {
            m_trajTimer.restart();
            m_forwardTimer = 0.0;
          } else {
            m_forwardTimer = (m_reverseTimer - m_trajTimer.get());
            m_reverseTimer = 0.0;
            m_trajTimer.restart();
          }
          m_currentState = ArmState.AMP;
        }

        double time = m_trajTimer.get() + m_forwardTimer;
        Logger.recordOutput("Arm/Actual Forward Time", time);

        armState = traj.sample(time);
        m_desiredArmPoseDegs = armState.armPositionDegs();
        m_desiredWristPoseDegs = armState.wristPositionDegs();

        m_armVelocityMult = armState.armVelocityDegsPerSec();
        m_wristVelocityMult = armState.wristVelocityDegsPerSec();
      }
      case AMP_REVERSE -> {
        ArmTrajectory traj = ArmConstants.AMP_TRAJECTORY;
        // if the arm is at the setpoint, then the arm is up at amp
        m_currentState = ArmState.AMP_REVERSE;
        if (m_reverseTimer == 0.0) {
          // restart the timer and find how long the last trajectory ran
          m_trajTimer.stop();
          m_reverseTimer = Math.min(m_trajTimer.get() + m_forwardTimer, traj.getFinalTime());
          m_trajTimer.restart();
        } else {
          // reverse the time by getting the difference between the trajectories length and the timer
          double time = m_reverseTimer - m_trajTimer.get();

          if (time <= 0.0) {
            m_reverseTimer = 0.0;
            m_trajTimer.stop();
            m_trajTimer.reset();
            armState = null;
            m_desiredState = ArmState.STOW;
            handleState();
          }

          // get the current state in the trajectory
          armState =
              traj.sample(time);

          m_desiredArmPoseDegs = armState.armPositionDegs();
          m_armVelocityMult = armState.armVelocityDegsPerSec();

          m_desiredWristPoseDegs = armState.wristPositionDegs();
          m_wristVelocityMult = armState.wristVelocityDegsPerSec();
        }
      }
      case PREPARE_TRAP -> {
        m_desiredArmPoseDegs = ArmSetpoints.TRAP_PREPARE.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.TRAP_PREPARE.wristAngle();

        if (m_currentState == ArmState.SCORE_TRAP) {
          m_wristVelocityMult = m_inputs.armPositionDegs < 55.0 ?
              0.0 : 0.25;
          m_armVelocityMult = 0.25;

          if (bothAtSetpoint()) {
            m_currentState = ArmState.PREPARE_TRAP;
          }
        } else {
          if (Math.abs(m_inputs.wristPositionDegs - m_desiredWristPoseDegs) > 5.0) {
            m_armVelocityMult = 0.5;
          } else {
            m_armVelocityMult = 1.0;
          }
          m_wristVelocityMult = 1.0;
          m_currentState = ArmState.PREPARE_TRAP;
        }
      }
      case SCORE_TRAP -> {
        if (m_currentState != ArmState.PREPARE_TRAP && m_currentState != ArmState.SCORE_TRAP) {
          m_desiredState = ArmState.STOW;
          handleState();
        }

        m_wristVelocityMult = 0.10;
        m_armVelocityMult = 0.10;

        m_currentState = ArmState.SCORE_TRAP;

        m_desiredArmPoseDegs = ArmSetpoints.TRAP_SCORE.armAngle();
        m_desiredWristPoseDegs = ArmSetpoints.TRAP_SCORE.wristAngle();
      }
      case PASS ->  {
        m_desiredWristPoseDegs = 45.0;
        m_desiredArmPoseDegs = 0.0;
        m_currentState = ArmState.PASS;
      }
      case BACKUP_SHOT -> {
        m_desiredWristPoseDegs = 50.0;
        m_desiredArmPoseDegs = 0.0;
        m_currentState = ArmState.BACKUP_SHOT;
      }
      case MANUAL_CONTROL -> {
        m_desiredWristPoseDegs = ArmSetpoints.STATIC_SHOOTER.wristAngle();

        m_desiredArmPoseDegs = ArmSetpoints.STATIC_SHOOTER.armAngle();
        m_currentState = ArmState.MANUAL_CONTROL;
      }
      default -> {
        m_currentState = ArmState.DISABLED;
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
    return Math.abs(m_inputs.armPositionDegs - m_desiredArmPoseDegs) < 7.5;
  }

  public boolean wristAtSetpoint() {
    return Math.abs(m_inputs.wristPositionDegs - m_desiredWristPoseDegs) < 7.5;
  }

  public boolean bothAtSetpoint() {
    return armAtSetpoint() && wristAtSetpoint();
  }

  public ArmState getArmState() {
    return m_currentState;
  }

  /* Command Factories */

  public Command setDesiredStateFactory(ArmState state) {
    return startEnd(() -> m_desiredState = state,
        () -> m_desiredState = ArmState.STOW);
  }

  public Command enableBrakeMode(boolean enabled) {
    return runOnce(() -> m_disabledBrakeMode = enabled).ignoringDisable(true);
  }

  public Command resetEncoderFactory() {
    return runOnce(m_io::resetPosition).ignoringDisable(true);
  }

  public Command incrementArmManual(double increment) {
    return runOnce(() -> {
      m_desiredState = ArmState.MANUAL_CONTROL;
      m_armIncremental += increment;
    });
  }

  public Command incrementWristManual(double increment) {
    return runOnce(() -> {
      m_desiredState = ArmState.MANUAL_CONTROL;
      m_wristIncremental += increment;
    });
  }
}
