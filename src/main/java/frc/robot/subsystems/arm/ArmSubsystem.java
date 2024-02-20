package frc.robot.subsystems.arm;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs;
  private double m_desiredArmPose;
  private double m_desiredWristPose;
  private final boolean useMM = false;

  private final SysIdRoutine m_sysidArm;
  private final SysIdRoutine m_sysidWrist;

  public ArmSubsystem(ArmIO io) {
    m_io = io;
    m_inputs = new ArmIOInputsAutoLogged();

    m_desiredWristPose = -1;
    m_desiredArmPose = -1;

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
                  .angularPosition(mutable(Rotations.of(m_inputs.armPositionRots)))
                  .angularVelocity(mutable(RotationsPerSecond.of(m_inputs.armVelocityRotsPerSecond))),
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
                  .angularPosition(mutable(Rotations.of(m_inputs.wristPositionRots)))
                  .angularVelocity(mutable(RotationsPerSecond.of(m_inputs.wristVelocityRotsPerSecond))),
              this));
    }
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

    // check to make sure we're not in manual control
    if (m_desiredWristPose != -1 && m_desiredArmPose != -1) {
      m_io.setArmAngle(m_desiredArmPose, useMM);

      // check to see if the wrist is currently too close to the rest of the arm
      if ((m_inputs.wristPositionRots * 360) - (360 - (m_inputs.armPositionRots * 360)) < 15) {
        m_io.setWristAngle(m_inputs.wristPositionRots * 360, useMM);
      } else {
        m_io.setWristAngle(m_desiredWristPose, useMM);
      }
    } else if (m_desiredWristPose != -2 && m_desiredArmPose != -2) {
      // check for stopped motors, if not hold position (still in manual mode
      m_io.setWristAngle(m_inputs.wristPositionRots, useMM);
      m_io.setArmAngle(m_inputs.armPositionRots, useMM);
    } else {
      m_io.stop();
      // keep us in stop mode until told otherwise
      m_desiredArmPose = -2;
      m_desiredWristPose = -2;
    }
  }

  public Command setArmDesiredPose(double armPose, double wristPose) {
    return runOnce(() -> {
      m_desiredArmPose = armPose;
      m_desiredWristPose = wristPose;
    });
  }

  public Command setArmPowerFactory(double power) {
    m_desiredArmPose = -1;
    return runOnce(() -> m_io.setArmVoltage(power * 12.0));
  }

  public Command setArmPositionFactory(double degrees) {
    return runOnce(() -> m_desiredArmPose = degrees);
  }

  public Command setWristPowerFactory(double power) {
    m_desiredWristPose = -1;
    return runOnce(() -> m_io.setWristVoltage(power * 12.0));
  }

  public Command setWristPositionFactory(double degrees) {
    return runOnce(() -> m_desiredWristPose = degrees);
  }

  public Command stopArmFactory() {
    m_desiredWristPose = -2;
    m_desiredArmPose = -2;
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
}

