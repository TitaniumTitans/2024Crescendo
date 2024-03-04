package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
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

    m_leftSetpoint = new LoggedDashboardNumber("Shooter/Left Flywheel Setpoint RPM", 5000);
    m_rightSetpoint = new LoggedDashboardNumber("Shooter/Right Flywheel Setpoint RPM", 5000);
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

  public Command runShooterVelocity(boolean runKicker) {
    return runEnd(() -> {
          m_io.setLeftVelocityRpm(m_leftSetpoint.get());
          m_io.setRightVelocityRpm(m_rightSetpoint.get());

          if ( /*(Math.abs(m_leftSetpoint.get() - m_inputs.tlVelocityRPM) < 15
          || Math.abs(m_rightSetpoint.get() - m_inputs.trVelocityRPM) < 15)
          && */ runKicker) {
            m_io.setKickerVoltage(3.0);
            m_io.setIntakeVoltage(0.05);
          } else {
            m_io.setKickerVoltage(0.0);
            m_io.setIntakeVoltage(0.0);
          }
        },
        () -> {
          m_io.setMotorVoltageTL(0.0);
          m_io.setMotorVoltageTR(0.0);
          m_io.setKickerVoltage(0.0);
          m_io.setIntakeVoltage(0.0);
        });

  }

  public boolean hasPiece() {
    return m_inputs.tofDistanceIn < 60;
  }

  /** Command Factories */

  public Command intakeCommand(double intakePower, double kickerPower, double timeout) {
    Timer timer = new Timer();
    return runEnd(() -> {
          if (!hasPiece()) {
            setShooterPowerLeft(-0.1);
            setShooterPowerRight(-0.1);
            setIntakePower(intakePower);
            setKickerPower(kickerPower);
            timer.restart();
          } else if (!timer.hasElapsed(timeout)) {
            setShooterPowerLeft(0.0);
            setShooterPowerRight(0.0);
            setKickerPower(-0.25);
            setIntakePower(0.0);
          } else {
            setKickerPower(0.0);
            setIntakePower(0.0);
          }
        },
        () -> {
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

