package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

public class ShooterIOPrototype implements ShooterIO {
  private final CANSparkFlex m_topLeftMotor;
  private final CANSparkFlex m_topRightMotor;
  private final CANSparkMax m_bottomLeftMotor;
  private final CANSparkMax m_bottomRightMotor;
  private final CANSparkMax m_kickekMotor;
  private final CANSparkMax m_intakeLeft;
  private final CANSparkMax m_intakeRight;
  public ShooterIOPrototype() {
    m_topLeftMotor = new CANSparkFlex(13, CANSparkLowLevel.MotorType.kBrushless);
    m_topRightMotor = new CANSparkFlex(14, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomLeftMotor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomRightMotor = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);
    m_kickekMotor = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeLeft = new CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeRight = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);

    m_topLeftMotor.setInverted(false);
    m_bottomLeftMotor.setInverted(true);

    m_intakeRight.setInverted(true);
    m_intakeLeft.setInverted(false);

    m_topLeftMotor.burnFlash();
    m_topRightMotor.burnFlash();
    m_intakeLeft.burnFlash();
    m_intakeRight.burnFlash();
  }

  @Override
  public void setMotorVoltageTL(double voltage) {
    m_topLeftMotor.setVoltage(voltage);
  }

  @Override
  public void setMotorVoltageTR(double voltage) {
    m_topRightMotor.setVoltage(voltage);
  }

  @Override
  public void setMotorVoltageBL(double voltage) {
    m_bottomLeftMotor.setVoltage(voltage);
  }

  @Override
  public void setMotorVoltageBR(double voltage) {
    m_bottomRightMotor.setVoltage(voltage);
  }

  @Override
  public void setKickerVoltage(double voltage) {
    m_kickekMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    m_intakeLeft.setVoltage(voltage);
    m_intakeRight.setVoltage(voltage);
  }
}
