package frc.robot.subsystems.shooter;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO m_io;

  private static final String TOP_WHEEL_RATIO = "Top wheel ratio";
  public ShooterSubsystem(ShooterIO io) {
    m_io = io;
    SmartDashboard.putNumber(TOP_WHEEL_RATIO, 0.8);
    SmartDashboard.putNumber("Left wheel", 0.6);
    SmartDashboard.putNumber("Right wheel", 0.6);
  }

  public void setShooterPowerLeft(double power) {
    m_io.setVelocityTL((power * 12.0) * SmartDashboard.getNumber(TOP_WHEEL_RATIO, 0.85));
    m_io.setVelocityBL(power * 12.0);
  }

  public void setShooterPowerRight(double power) {
    m_io.setVelocityTR((power * 12.0) * SmartDashboard.getNumber(TOP_WHEEL_RATIO, 0.85));
    m_io.setVelocityBR(power * 12.0);
  }

  public void setKickerPower(double power) {
    m_io.setKickerVoltage(power * 12.0);
  }

  public void setIntakePower(double power) {
    m_io.setIntakeVoltage(power * 12.0);
  }

  public Command setShooterPowerFactory(double left, double right) {
    return run(() -> {
      setShooterPowerLeft(left == 0.0 ? 0.0 : SmartDashboard.getNumber("Left wheel", 0.6));
      setShooterPowerRight(right == 0.0 ? 0.0 : SmartDashboard.getNumber("Right wheel", 0.6));
      setKickerPower(left == 0.0 ? 0.0 : 1.0);
    });
  }

  public Command setIntakePowerFactory(double power) {
    return run(() -> {
      setIntakePower(power);
      setKickerPower(power == 0.0 ? 0.0 : 0.75);
    });
  }
}

