package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.BaseUnits.Voltage;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs;

  private final LoggedDashboardNumber m_leftSetpoint;
  private final LoggedDashboardNumber m_rightSetpoint;

  private final SysIdRoutine m_sysIdLeft;
  private final SysIdRoutine m_sysIdRight;

  public ShooterSubsystem(ShooterIO io) {
    m_io = io;
    m_inputs = new ShooterIOInputsAutoLogged();

    m_leftSetpoint = new LoggedDashboardNumber("Shooter/Left Flywheel Setpoint RPM");
    m_rightSetpoint = new LoggedDashboardNumber("Shooter/Right Flywheel Setpoint RPM");

    if (m_io.getClass() == ShooterIOKraken.class) {
      m_sysIdLeft = new SysIdRoutine(
          new SysIdRoutine.Config(null,
              Voltage.of(9),
              null,
              (SysIdRoutineLog.State state) -> SignalLogger.writeString("shooter-left-state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volt) -> setShooterPowerLeft(volt.in(Volts) / 12.0),
              null,
              this));
      m_sysIdRight = new SysIdRoutine(
          new SysIdRoutine.Config(null,
              Voltage.of(9),
              null,
              (SysIdRoutineLog.State state) -> SignalLogger.writeString("shooter-right-state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volt) -> setShooterPowerRight(volt.in(Volts) / 12.0),
              null,
              this));
    } else {
      m_sysIdLeft = new SysIdRoutine(
          new SysIdRoutine.Config(null, Voltage.of(9), null),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volt) -> setShooterPowerLeft(volt.in(Volts) / 12.0),
              (SysIdRoutineLog log) -> log.motor("shooter-left")
                  .voltage(mutable(Volts.of(m_inputs.tlAppliedVolts)))
                  .angularVelocity(mutable(RotationsPerSecond.of(m_inputs.tlVelocityRPM * 60.0))),
              this));
      m_sysIdRight = new SysIdRoutine(
          new SysIdRoutine.Config(null, Voltage.of(9), null),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volt) -> setShooterPowerRight(volt.in(Volts) / 12.0),
              (SysIdRoutineLog log) -> log.motor("shooter-right")
                  .voltage(mutable(Volts.of(m_inputs.trAppliedVolts)))
                  .angularVelocity(mutable(RotationsPerSecond.of(m_inputs.trVelocityRPM * 60.0))),
              this));
    }
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);
  }

  public void setShooterPowerLeft(double power) {
    m_io.setMotorVoltageTL(power * 12.0);
    m_io.setMotorVoltageBL(power * 12.0);
  }

  public void setShooterPowerRight(double power) {
    m_io.setMotorVoltageTR(power * 12.0);
    m_io.setMotorVoltageBR(power * 12.0);
  }

  public void setKickerPower(double power) {
    m_io.setKickerVoltage(power * 12.0);
  }

  public void setIntakePower(double power) {
    m_io.setIntakeVoltage(power * 12.0);
  }

  public void runShooterVelocity() {
    m_io.setLeftVelocityRpm(m_leftSetpoint.get());
    m_io.setRightVelocityRpm(m_rightSetpoint.get());
//    m_io.setKickerVoltage(9.0);
  }

  public Command setShooterPowerFactory(double left, double right) {
    return run(() -> {
      setShooterPowerLeft(left);
      setShooterPowerRight(right);
      setKickerPower(left == 0.0 ? 0.0 : 1.0);
      setIntakePower(left == 0.0 ? 0.0 : 0.35);
    });
  }
}

