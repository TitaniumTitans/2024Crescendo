package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;

public class ArmIOPrototype implements ArmIO {
    private final TalonFX m_shoulder;
    private final TalonFX m_wrist;
    private final PidPropertyPublic m_shoulderPID;
    private final PidPropertyPublic m_wristPID;

    public ArmIOPrototype() {
        m_shoulder = new TalonFX(23);
        m_wrist = new TalonFX(24);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = 125.0 * (18.0 / 60.0);
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = -3;

        m_shoulder.getConfigurator().apply(config);

        config.Feedback.SensorToMechanismRatio = 125.0 * (18.0 / 38.0);
        m_wrist.getConfigurator().apply(config);

        m_wrist.stopMotor();
        m_shoulder.stopMotor();

        m_shoulderPID = new Phoenix6PidPropertyBuilder(
                "Arm/Shoulder PID",
                false,
                m_shoulder,
                0)
                .addP(0.0)
                .addI(0.0)
                .addD(0.0)
                .build();

        m_wristPID = new Phoenix6PidPropertyBuilder(
                "Arm/Wrist PID",
                false,
                m_wrist,
                0)
                .addP(0.0)
                .addI(0.0)
                .addD(0.0)
                .build();

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        m_shoulderPID.updateIfChanged();
        m_wristPID.updateIfChanged();
    }

    @Override
    public void setShoulderVoltage(double voltage){
        m_shoulder.setVoltage(voltage);
    }

    @Override
    public void setShoulderAngle(double degrees) {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        m_shoulder.setControl(m_request.withPosition(degrees / 360.0));
    }

    @Override
    public void setWristVoltage(double voltage){
        m_wrist.setVoltage(voltage);
    }

    @Override
    public void setWristAngle(double degrees) {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        m_wrist.setControl(m_request.withPosition(degrees / 360.0));
    }

    @Override
    public Rotation2d getWristPosition() {
        return Rotation2d.fromRotations(m_wrist.getPosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getShoulderPosition() {
        return Rotation2d.fromRotations(m_wrist.getPosition().getValueAsDouble());
    }
}
