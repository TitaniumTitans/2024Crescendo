package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class ShooterIntakeIOPrototype implements ShooterIO{
  private final CANSparkMax m_leftIntake;
  private final CANSparkMax m_rightIntake;

  public ShooterIntakeIOPrototype() {
    m_leftIntake = new CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless);
    m_rightIntake = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);

    m_rightIntake.setInverted(true);
    m_leftIntake.setInverted(true);

    m_rightIntake.burnFlash();
    m_leftIntake.burnFlash();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    m_leftIntake.setVoltage(voltage);
    m_rightIntake.setVoltage(voltage);
  }
}
