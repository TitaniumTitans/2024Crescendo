package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.BaseUnits.Voltage;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs;

  private final LoggedDashboardNumber m_leftSetpoint;
  private final LoggedDashboardNumber m_rightSetpoint;

  public ShooterSubsystem(ShooterIO io) {
    m_io = io;
    m_inputs = new ShooterIOInputsAutoLogged();

    m_leftSetpoint = new LoggedDashboardNumber("Shooter/Left Flywheel Setpoint RPM");
    m_rightSetpoint = new LoggedDashboardNumber("Shooter/Right Flywheel Setpoint RPM");
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
    m_io.setKickerVoltage(9.0);
    m_io.setIntakeVoltage(7.5);
  }

  public boolean hasPiece() {
    return m_inputs.tofDistanceIn < 60;
  }

  /** Command Factories */

  public Command intakeCommand(double intakePower, double kickerPower, double timeout) {
    return (run(() -> {
      setIntakePower(intakePower);
      setKickerPower(kickerPower);
    }).until(this::hasPiece)
        .andThen(() -> {
          setIntakePower(intakePower * 0.5);
          setKickerPower(kickerPower);
        })
          .withTimeout(timeout)
        .andThen(() -> setKickerPower(kickerPower * -0.25))
          .withTimeout(timeout * 0.5)
        .finallyDo(() -> {
          setIntakePower(0.0);
          setKickerPower(0.0);
        })
        )
        .handleInterrupt(() -> {
          setIntakePower(0.0);
          setKickerPower(0.0);
        });
  }

  public Command setShooterPowerFactory(double left, double right, double intake) {
    return run(() -> {
      setShooterPowerLeft(left);
      setShooterPowerRight(right);
      setKickerPower(intake);
      setIntakePower(intake);
    });
  }
}

