package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO m_io;
  private final ClimberIOInputsAutoLogged m_inputs;
  public ClimberSubsystem(ClimberIO io) {
    m_io = io;
    m_inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Climber", m_inputs);
  }

  public void setClimberPower(double power) {
    m_io.setRightVoltage(power * 12.0);
    m_io.setLeftVoltage(power * 12.0);
  }

  public Command setClimberPowerFactory(double power) {
    return runEnd(() -> setClimberPower(power), () -> setClimberPower(0.0));
  }

  public Command setLeftClimberPowerFactory(double power) {
    return runEnd(() -> m_io.setLeftVoltage(power * 12.0), () -> setClimberPower(0.0));
  }

  public Command setRightClimberPowerFactory(double power) {
    return runEnd(() -> m_io.setRightVoltage(power * 12.0), () -> setClimberPower(0.0));
  }

  public Command setClimberPosition(double degrees) {
    return runEnd(() -> {
          m_io.setLeftPosition(degrees);
          m_io.setRightPosition(degrees);
        },
        () -> setClimberPower(0.0));
  }

  public Command resetClimber() {
    return runOnce(m_io::resetPosition);
  }
}

