package frc.robot.subsystems.shooter;

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.*;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterIOPrototype implements ShooterIO {
  private final CANSparkFlex m_topLeftMotor;
  private final CANSparkFlex m_topRightMotor;
  private final CANSparkFlex m_bottomLeftMotor;
  private final CANSparkFlex m_bottomRightMotor;
  private final CANSparkMax m_kickerMotor;

  private final SparkPIDController m_leftPid;
  private final PidProperty m_leftPidProperty;
  private final SparkPIDController m_rightPid;
  private final PidProperty m_rightPidProperty;

  public ShooterIOPrototype() {
    m_topLeftMotor = new CANSparkFlex(ShooterConstants.TOP_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_topRightMotor = new CANSparkFlex(ShooterConstants.TOP_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomLeftMotor = new CANSparkFlex(ShooterConstants.BOTTOM_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomRightMotor = new CANSparkFlex(ShooterConstants.BOTTOM_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_kickerMotor = new CANSparkMax(ShooterConstants.KICKER_ID, CANSparkLowLevel.MotorType.kBrushless);

    m_topLeftMotor.setInverted(ShooterConstants.TOP_LEFT_INVERTED);
    m_topRightMotor.setInverted(ShooterConstants.TOP_RIGHT_INVERTED);
    m_bottomLeftMotor.setInverted(ShooterConstants.BOTTOM_LEFT_INVERTED);
    m_bottomRightMotor.setInverted(ShooterConstants.BOTTOM_RIGHT_INVERTED);
    m_kickerMotor.setInverted(ShooterConstants.KICKER_INVERTED);

    m_topLeftMotor.enableVoltageCompensation(12);
    m_topRightMotor.enableVoltageCompensation(12);
    m_bottomLeftMotor.enableVoltageCompensation(12);
    m_bottomRightMotor.enableVoltageCompensation(12);

    m_bottomLeftMotor.follow(m_topLeftMotor);
    m_bottomRightMotor.follow(m_topRightMotor);

    m_leftPid = m_topLeftMotor.getPIDController();
    m_leftPidProperty = new RevPidPropertyBuilder("Shooter/Left Shooter", false, m_leftPid, 0)
                    .addP(ShooterConstants.SHOOTER_KP)
                    .addI(ShooterConstants.SHOOTER_KI)
                    .addD(ShooterConstants.SHOOTER_KD)
                    .addFF(0.0)
                    .build();
    m_rightPid = m_topLeftMotor.getPIDController();
    m_rightPidProperty = new RevPidPropertyBuilder("Shooter/Left Shooter", false, m_leftPid, 0)
                    .addP(ShooterConstants.SHOOTER_KP)
                    .addI(ShooterConstants.SHOOTER_KI)
                    .addD(ShooterConstants.SHOOTER_KD)
                    .addFF(0.0)
                    .build();

    m_topLeftMotor.burnFlash();
    m_topRightMotor.burnFlash();
    m_topRightMotor.burnFlash();
    m_bottomRightMotor.burnFlash();
    m_kickerMotor.burnFlash();
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
    m_kickerMotor.setVoltage(voltage);
  }

  @Override
  public void setLeftVelocityRpm(double rpm) {
    m_leftPid.setReference(rpm, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void setRightVelocityRpm(double rpm) {
    m_rightPid.setReference(rpm, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_leftPidProperty.updateIfChanged();
    m_rightPidProperty.updateIfChanged();

    inputs.tlVelocityRads = m_topLeftMotor.getEncoder().getVelocity() * Math.PI * 2.0;
    inputs.trVelocityRads = m_topRightMotor.getEncoder().getVelocity() * Math.PI * 2.0;
    inputs.blVelocityRads = m_bottomLeftMotor.getEncoder().getVelocity() * Math.PI * 2.0;
    inputs.brVelocityRads = m_bottomRightMotor.getEncoder().getVelocity() * Math.PI * 2.0;

    inputs.tlAppliedVolts = m_topLeftMotor.getAppliedOutput() * m_topLeftMotor.getBusVoltage();
    inputs.trAppliedVolts = m_topRightMotor.getAppliedOutput() * m_topRightMotor.getBusVoltage();
    inputs.blAppliedVolts = m_bottomLeftMotor.getAppliedOutput() * m_bottomLeftMotor.getBusVoltage();
    inputs.brAppliedVolts = m_bottomRightMotor.getAppliedOutput() * m_bottomRightMotor.getBusVoltage();
    inputs.kickerAppliedVolts = m_kickerMotor.getAppliedOutput() * m_kickerMotor.getBusVoltage();

    inputs.tlCurrentDraw = m_topLeftMotor.getOutputCurrent();
    inputs.trCurrentDraw = m_topRightMotor.getOutputCurrent();
    inputs.blCurrentDraw = m_bottomLeftMotor.getOutputCurrent();
    inputs.brCurrentDraw = m_bottomRightMotor.getOutputCurrent();
    inputs.kickerCurrentDraw = m_kickerMotor.getOutputCurrent();

    inputs.tlTemperature = m_topLeftMotor.getMotorTemperature();
    inputs.trTemperature = m_topRightMotor.getMotorTemperature();
    inputs.blTemperature = m_bottomLeftMotor.getMotorTemperature();
    inputs.brTemperature = m_bottomRightMotor.getMotorTemperature();
  }
}
