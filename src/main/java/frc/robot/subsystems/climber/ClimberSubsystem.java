package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO m_io;
  private final ClimberIOInputsAutoLogged m_inputs;

  private boolean m_climberLock = false;

  public ClimberSubsystem(ClimberIO io) {
    m_io = io;
    m_inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Climber", m_inputs);
    Logger.recordOutput("Climber/Climber Lock", m_climberLock);
  }

  public void setClimberPower(double power) {
    m_climberLock = true;
    m_io.setRightVoltage(power * 12.0);
    m_io.setLeftVoltage(power * 12.0);
  }

  public boolean getClimberLock() {
    return m_climberLock;
  }

  public Command resetClimberLock() {
    return runOnce(() -> m_climberLock = false).ignoringDisable(true);
  }

  public Command setClimberPowerFactory(double power) {
    m_climberLock = true;
    return runEnd(() -> setClimberPower(power), () -> setClimberPower(0.0));
  }

  public Command setLeftClimberPowerFactory(double power) {
    m_climberLock = true;
    return runEnd(() -> m_io.setLeftVoltage(power * 12.0), () -> setClimberPower(0.0));
  }

  public Command setRightClimberPowerFactory(double power) {
    m_climberLock = true;
    return runEnd(() -> m_io.setRightVoltage(power * 12.0), () -> setClimberPower(0.0));
  }

  public Command setClimberPosition(double degrees) {
    m_climberLock = true;
    return runEnd(() -> {
          m_io.setLeftPosition(degrees);
          m_io.setRightPosition(degrees);
        },
        () -> setClimberPower(0.0));
  }

  public Command resetClimber() {
    return runOnce(m_io::resetPosition);
  }

  public double getClimberHeight() {
    return (m_inputs.getLeftClimberPosition() + m_inputs.getRightClimberPosition()) / 2.0;
  }
}

