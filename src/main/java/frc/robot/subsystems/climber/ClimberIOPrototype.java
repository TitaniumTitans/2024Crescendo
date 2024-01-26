package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;

public class ClimberIOPrototype implements ClimberIO {
    private final TalonFX m_leftTalon;
    private final TalonFX m_rightTalon;
    private final PidPropertyPublic m_leftClimberPid;
    private final PidPropertyPublic m_rightClimberPid;

    public ClimberIOPrototype() {
        m_leftTalon = new TalonFX(18);
        m_rightTalon = new TalonFX(19);

        var rightTalonConfig = new TalonFXConfiguration();
        rightTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_rightTalon.getConfigurator().apply(rightTalonConfig);

        var leftTalonConfig = new TalonFXConfiguration();
        m_leftTalon.getConfigurator().apply(leftTalonConfig);

        m_leftClimberPid = new Phoenix6PidPropertyBuilder(
                "Climber/Left PID",
                false, m_leftTalon, 0)
                .addP(0.0)
                .addI(0.0)
                .addD(0.0)
                .build();

        m_rightClimberPid = new Phoenix6PidPropertyBuilder(
                "Climber/Right PID",
                false, m_rightTalon, 0)
                .addP(0.0)
                .addI(0.0)
                .addD(0.0)
                .build();

    }

    @Override
    public void setLeftPosition(double degrees) {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        m_leftTalon.setControl(m_request.withPosition(degrees / 360.0));
    }
    @Override
    public void setRightPosition(double degrees) {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        m_rightTalon.setControl(m_request.withPosition(degrees / 360.0));
    }
}
