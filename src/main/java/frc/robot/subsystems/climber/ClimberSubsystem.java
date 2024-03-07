package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO m_io;
  private final ClimberIO.ClimberIOInputs m_inputs;
  public ClimberSubsystem(ClimberIO io) {
    m_io = io;
    m_inputs = new ClimberIO.ClimberIOInputs();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
  }

  public void setClimberPower(double power) {
    m_io.setRightVoltage(power * 12.0);
    m_io.setLeftVoltage(power * 12.0);
  }

  public Command setClimberPowerFactory(double power) {
    return runEnd(() -> setClimberPower(power), () -> setClimberPower(0.0));
  }
}

