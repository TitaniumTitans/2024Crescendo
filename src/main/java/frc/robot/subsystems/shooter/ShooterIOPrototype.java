package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

public class ShooterIOPrototype implements ShooterIO {
  private final CANSparkFlex m_topLeftMotor;
  private final CANSparkMax m_topRightMotor;
  private final CANSparkFlex m_bottomLeftMotor;
  private final CANSparkMax m_bottomRightMotor;
  private final CANSparkMax m_kickerkMotor;
  public ShooterIOPrototype() {
    m_topLeftMotor = new CANSparkFlex(13, CANSparkLowLevel.MotorType.kBrushless);
    m_topRightMotor = new CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomLeftMotor = new CANSparkFlex(15, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomRightMotor = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);
    m_kickerkMotor = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless);

    m_topLeftMotor.setInverted(false);
    m_topRightMotor.setInverted(true);
    m_bottomLeftMotor.setInverted(true);
    m_bottomRightMotor.setInverted(true);
    m_kickerkMotor.setInverted(true);

    m_topLeftMotor.burnFlash();
    m_topRightMotor.burnFlash();
    m_topRightMotor.burnFlash();
    m_bottomRightMotor.burnFlash();
    m_kickerkMotor.burnFlash();
  }

//    @Override
//    public void updateInputs(ShooterIOInputs inputs) {
//        inputs.tLAngularVelocity = m_topLeftMotor.getEncoder().getVelocity() * Math.PI * 2;
//        inputs.tRAngularVelocity = m_topRightMotor.getEncoder().getVelocity() * Math.PI;
//        inputs.bLAngularVelocity = m_bottomLeftMotor.getEncoder().getVelocity() * Math.PI;
//        inputs.bRAngularVelocity = m_bottomRightMotor.getEncoder().getVelocity() * Math.PI;
//        inputs.kickerAngularVelocity = m_kickerMotor.getEncoder().getVelocity() * Math.PI;
//
//        inputs.tLAppliedInputs = m_topLeftMotor.getAppliedOutput();
//        inputs.tRAppliedInputs = m_topRightMotor.getAppliedOutput();
//        inputs.bLAppliedInputs = m_bottomLeftMotor.getAppliedOutput();
//        inputs.bRAppliedInputs = m_bottomRightMotor.getAppliedOutput();
//        inputs.kickerAppliedInputs = m_kickerMotor.getAppliedOutput();
//    }

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
    m_kickerkMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.tlVelocity = m_topLeftMotor.getEncoder().getVelocity() * Math.PI * 2.0;
    inputs.trVelocity = m_topRightMotor.getEncoder().getVelocity() * Math.PI * 2.0;
    inputs.blVelocity = m_bottomLeftMotor.getEncoder().getVelocity() * Math.PI * 2.0;
    inputs.brVelocity = m_bottomRightMotor.getEncoder().getVelocity() * Math.PI * 2.0;
    inputs.tlAppliedOutput = m_topLeftMotor.getAppliedOutput();
    inputs.trAppliedOutput = m_topRightMotor.getAppliedOutput();
    inputs.blAppliedOutput = m_bottomLeftMotor.getAppliedOutput();
    inputs.brAppliedOutput = m_bottomRightMotor.getAppliedOutput();
    inputs.kickerAppliedOutput = m_kickerkMotor.getAppliedOutput();
    inputs.tlCurrentDraw = m_topLeftMotor.getOutputCurrent();
    inputs.trCurrentDraw = m_topRightMotor.getOutputCurrent();
    inputs.blCurrentDraw = m_bottomLeftMotor.getOutputCurrent();
    inputs.brCurrentDraw = m_bottomRightMotor.getOutputCurrent();
    inputs.kickerCurrentDraw = m_kickerkMotor.getOutputCurrent();
    inputs.tlTemperature = m_topLeftMotor.getMotorTemperature();
    inputs.trTemperature = m_topRightMotor.getMotorTemperature();
    inputs.blTemperature = m_bottomLeftMotor.getMotorTemperature();
    inputs.brTemperature = m_bottomRightMotor.getMotorTemperature();
  }
}
