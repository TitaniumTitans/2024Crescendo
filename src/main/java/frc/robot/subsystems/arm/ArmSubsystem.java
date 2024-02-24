package frc.robot.subsystems.arm;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
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
  private static final boolean USE_MM = false;

  private final SysIdRoutine m_sysidArm;
  private final SysIdRoutine m_sysidWrist;

  public ArmSubsystem(ArmIO io) {
    m_io = io;
    m_inputs = new ArmIOInputsAutoLogged();

    m_desiredWristPoseDegs = -1;
    m_desiredArmPoseDegs = -1;

    if (io.getClass() == ArmIOKraken.class) { // when using a talon drive
      m_sysidArm = new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // ramp rate
              Volts.of(6),
              null, // timeout for safety
              // log state with phoenix logger
              (SysIdRoutineLog.State state) -> SignalLogger.writeString("arm-state", state.toString())
          ),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> m_io.setArmVoltage(volts.in(Volts)),
              null,
              this));

      m_sysidWrist = new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // ramp rate
              Volts.of(6),
              null, // timeout for safety
              // log state with phoenix logger
              (SysIdRoutineLog.State state) -> SignalLogger.writeString("wrist-state", state.toString())
          ),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> m_io.setWristVoltage(volts.in(Volts)),
              null,
              this));

    } else { // if not using a talon drive for some reason
      m_sysidArm = new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(6),
              null
          ),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> m_io.setArmVoltage(volts.in(Volts)),
              (SysIdRoutineLog log) -> log.motor("arm-pivot")
                  .voltage(mutable(Volts.of(m_inputs.armAppliedOutput * RobotController.getBatteryVoltage())))
                  .angularPosition(mutable(Degrees.of(m_inputs.armPositionDegs)))
                  .angularVelocity(mutable(DegreesPerSecond.of(m_inputs.armVelocityDegsPerSecond))),
              this));

      m_sysidWrist = new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(6),
              null
          ),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> m_io.setWristVoltage(volts.in(Volts)),
              (SysIdRoutineLog log) -> log.motor("wrist-pivot")
                  .voltage(mutable(Volts.of(m_inputs.wristAppliedOutput * RobotController.getBatteryVoltage())))
                  .angularPosition(mutable(Degrees.of(m_inputs.wristPositionDegs)))
                  .angularVelocity(mutable(DegreesPerSecond.of(m_inputs.wristVelocityDegsPerSecond))),
              this));
    }

    m_io.resetPosition();
  }

  //TODO: Finite state machine logic

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

    // clamp values for PID inbetween acceptable ranges
    m_desiredWristPoseDegs = m_desiredWristPoseDegs > 0 ?
        MathUtil.clamp(m_desiredWristPoseDegs, ArmConstants.WRIST_LOWER_LIMIT.getValue(),
            ArmConstants.WRIST_UPPER_LIMIT.getValue())
        : m_desiredWristPoseDegs;

    m_desiredArmPoseDegs = m_desiredArmPoseDegs > 0 ?
        MathUtil.clamp(m_desiredArmPoseDegs, ArmConstants.ARM_LOWER_LIMIT.getValue(),
            ArmConstants.ARM_UPPER_LIMIT.getValue())
        : m_desiredArmPoseDegs;

    // check to make sure we're not in manual control
    if (m_desiredWristPoseDegs > -1 && m_desiredArmPoseDegs > -1) {
      m_io.setArmAngle(m_desiredArmPoseDegs, USE_MM);

      // check to see if the wrist is currently too close to the rest of the arm
      if (m_inputs.wristPositionDegs + m_inputs.armPositionDegs
          < ArmConstants.WRIST_ARM_GAP.getValue()) {
        m_io.setWristAngle(m_inputs.wristPositionDegs, USE_MM);
      } else {
        m_io.setWristAngle(m_desiredWristPoseDegs, USE_MM);
      }
//    } else if (m_desiredWristPoseDegs != -2 && m_desiredArmPoseDegs != -2) {
//       check for stopped motors, if not hold position (still in manual mode)
//      m_io.setWristAngle(m_inputs.wristPositionDegs, USE_MM);
//      m_io.setArmAngle(m_inputs.armPositionDegs, USE_MM);
//    } else {
//      m_io.stop();
//       keep us in stop mode until told otherwise
//      m_desiredArmPoseDegs = -2;
//      m_desiredWristPoseDegs = -2;
    }
  }

  public Command setArmDesiredPose(double armPose, double wristPose) {
    return runOnce(() -> {
      m_desiredArmPoseDegs = armPose;
      m_desiredWristPoseDegs = wristPose;
    });
  }

  public Command setArmPowerFactory(double power) {
    // let arm know it's in manual control
    m_desiredArmPoseDegs = -1;
    // limiting code for arm
    if (m_inputs.armPositionDegs > ArmConstants.ARM_UPPER_LIMIT.getValue()) {
      power = MathUtil.clamp(power, -1.0, 0.0);
    } else if (m_inputs.armPositionDegs < ArmConstants.ARM_LOWER_LIMIT.getValue()) {
      power = MathUtil.clamp(power, 0.0, 1.0);
    }
    double finalPower = power;
    return runOnce(() -> m_io.setArmVoltage(finalPower * 12.0));
  }

  public Command setArmPositionFactory(double degrees) {
    return runOnce(() -> m_desiredArmPoseDegs = degrees);
  }

  public Command setWristPowerFactory(double power) {
    // let wrist know it's in manual control mode
    m_desiredWristPoseDegs = -1;
    // limiting code for wrist
    if (m_inputs.wristPositionDegs > ArmConstants.WRIST_UPPER_LIMIT.getValue()) {
      power = MathUtil.clamp(power, -1.0, 0.0);
    }
    if (m_inputs.wristPositionDegs < ArmConstants.WRIST_LOWER_LIMIT.getValue()) {
//        || m_inputs.wristPositionDegs + m_inputs.armPositionDegs < ArmConstants.WRIST_ARM_GAP.getValue()) {
      power = MathUtil.clamp(power, 0.0, 1.0);
    }
    double finalPower = power;
    return runOnce(() -> m_io.setWristVoltage(finalPower * 12.0));
  }

  public Command setWristPositionFactory(double degrees) {
    return runOnce(() -> m_desiredWristPoseDegs = degrees);
  }

  public Command stopArmFactory() {
    // put wrist and arm into stop mode to keep PID from running
    m_desiredWristPoseDegs = -2;
    m_desiredArmPoseDegs = -2;
    return runOnce(() -> {
      m_io.setWristVoltage(0.0);
      m_io.setArmVoltage(0.0);
    });
  }

  public Command sysIdArmDynamic(SysIdRoutine.Direction direction) {
    return m_sysidArm.dynamic(direction)
        .andThen(stopArmFactory());
  }

  public Command sysIdArmQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysidArm.quasistatic(direction)
        .andThen(stopArmFactory());
  }

  public Command sysIdWristDynamic(SysIdRoutine.Direction direction) {
    return m_sysidWrist.dynamic(direction)
        .andThen(stopArmFactory());
  }

  public Command sysIdWristQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysidWrist.quasistatic(direction)
        .andThen(stopArmFactory());
  }

  public Command resetEncoderFactory() {
    return runOnce(m_io::resetPosition);

  }
}

