package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO m_io;
    public ArmSubsystem(ArmIO io) {
        m_io = io;
    }

    public void setShoulderPower(double power){
        m_io.setShoulderVoltage(power * 12.0);
    }
    public void setWristPower(double power){
        m_io.setWristVoltage(power * 12);
    }

    public Command setShoulderPowerFactory(double power) {
        return runOnce(() -> setShoulderPower(power));
    }

    public Command setWristPowerFactory(double power) {
        return runOnce(() -> setWristPower(power));
    }
}

