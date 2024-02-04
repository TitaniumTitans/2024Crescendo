package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO m_io;
    public ArmSubsystem(ArmIO io) {
        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(new ArmIOInputsAutoLogged());

//        m_io.setWristVoltage(0.0);
//        m_io.setShoulderVoltage(0.0);
        SmartDashboard.putNumber("Recorded Wrist Position", m_io.getWristPosition().getDegrees());
        SmartDashboard.putNumber("Recorded Shoulder Position", m_io.getShoulderPosition().getDegrees());
    }

    public void setShoulderPower(double power) {
        m_io.setShoulderVoltage(power * 12.0);
    }

    public void setShoulderPosition(double degrees) {
        m_io.setShoulderAngle(degrees);
    }

    public void setWristPower(double power) {m_io.setWristVoltage(power * 12.0);}

    public void setWristPosition(double degrees) {
        m_io.setWristAngle(degrees);
    }

    public Command setShoulderPowerFactory(double power) {
        return runOnce(() -> setShoulderPower(power));
    }

    public Command setShoulderPositionFactory(double degrees) {
        return runOnce(() -> setShoulderPosition(degrees));
    }

    public Command setWristPowerFactory(double power) {
        return runOnce(() -> setWristPower(power));
    }

    public Command setWristPositionFactory(double degrees) {
        return runOnce(() -> setWristPosition(degrees));
    }

    public Command stopArmFactory() {
        return runOnce(() -> {
            setWristPower(0.0);
            setShoulderPower(0.0);
        });
    }
}

