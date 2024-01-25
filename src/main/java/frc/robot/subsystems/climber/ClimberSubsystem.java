package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO m_io;
    public ClimberSubsystem(ClimberIO io) {
        m_io = io;
    }

    public void setClimberPower(double power) {
        m_io.setLeftVoltage(power * 12.0);
        m_io.setRightVoltage(power * 12.0);
    }

    public Command setClimberPowerFactory(double power) {
        return run(() -> setClimberPower(power));
    }
}

