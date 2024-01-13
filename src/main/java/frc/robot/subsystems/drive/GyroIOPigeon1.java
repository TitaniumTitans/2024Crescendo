package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon1 implements GyroIO {
    private final PigeonIMU m_pigeon;
    private double prevYaw;

    public GyroIOPigeon1(int id, boolean pheonixDrive) {
        m_pigeon = new PigeonIMU(id);
        m_pigeon.configFactoryDefault();
        m_pigeon.setYaw(0.0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = m_pigeon.getState() == PigeonIMU.PigeonState.Ready;
        inputs.yawPosition = Rotation2d.fromDegrees(m_pigeon.getYaw());
        inputs.yawVelocityRadPerSec = (m_pigeon.getYaw() - prevYaw) / Units.millisecondsToSeconds(20);
        prevYaw = m_pigeon.getYaw();
    }
}
