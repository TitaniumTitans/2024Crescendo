package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;

public class ClimberIOPrototype implements ClimberIO {
    private final TalonFX m_leftTalon;
    private final TalonFX m_rightTalon;

    public ClimberIOPrototype() {
        m_leftTalon = new TalonFX(18);
        m_rightTalon = new TalonFX(19);

        var rightTalonConfig = new TalonFXConfiguration();
        rightTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_rightTalon.getConfigurator().apply(rightTalonConfig);

        var leftTalonConfig = new TalonFXConfiguration();
        m_leftTalon.getConfigurator().apply(leftTalonConfig);
    }

    @Override
    public void setRightVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
        m_rightTalon.setVoltage(clampedVoltage);
    }

    @Override
    public void setLeftVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
        m_leftTalon.setVoltage(clampedVoltage);
    }
}
