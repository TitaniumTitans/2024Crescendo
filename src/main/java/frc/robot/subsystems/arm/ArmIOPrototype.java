package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmIOPrototype implements ArmIO {
    private final TalonFX m_shoulder;
    private final TalonFX m_wrist;
    public ArmIOPrototype() {
        m_shoulder = new TalonFX(23);
        m_wrist = new TalonFX(24);

    }

    @Override
    public void setShoulderVoltage(double voltage){
        m_shoulder.setVoltage(voltage);
    }

    @Override
    public void setWristVoltage(double voltage){
        m_wrist.setVoltage(voltage);
    }
}