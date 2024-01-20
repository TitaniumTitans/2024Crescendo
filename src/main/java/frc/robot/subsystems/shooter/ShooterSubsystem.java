package frc.robot.subsystems.shooter;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO m_io;
    public ShooterSubsystem(ShooterIO io) {
        m_io = io;
        SmartDashboard.putNumber("Top wheel ratio", 0.8);
        SmartDashboard.putNumber("Left wheel", 0.6);
        SmartDashboard.putNumber("Right wheel", 0.6);
    }

    public void setShooterPowerLeft(double power) {
        m_io.setMotorVoltageTL((power * 12.0) * SmartDashboard.getNumber("Top wheel ratio", 0.85));
        m_io.setMotorVoltageBL(power * 12.0);
    }

    public void setShooterPowerRight(double power) {
        m_io.setMotorVoltageTR((power * 12.0) * SmartDashboard.getNumber("Top wheel ratio", 0.85));
        m_io.setMotorVoltageBR(power * 12.0);
    }

    public void setKickerPower(double power) {
        m_io.setKickerVoltage(power * 12.0);
    }

    public Command setShooterPower(double left, double right) {
        return run(() -> {
            setShooterPowerLeft(left == 0.0 ? 0.0 : SmartDashboard.getNumber("Left wheel", 0.6));
            setShooterPowerRight(right == 0.0 ? 0.0 : SmartDashboard.getNumber("Right wheel", 0.6));
            setKickerPower(left == 0.0 ? 0.0 : 1.0);
        });
    }
}

