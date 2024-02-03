package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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

