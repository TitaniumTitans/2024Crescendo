package frc.robot.subsystems.shooter;

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.*;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOPrototype implements ShooterIO {
  private final CANSparkFlex m_topLeftMotor;
  private final CANSparkFlex m_topRightMotor;
  private final CANSparkFlex m_bottomLeftMotor;
  private final CANSparkFlex m_bottomRightMotor;
  private final CANSparkMax m_kickerMotor;

  private final SparkPIDController m_topLeftPid;
  private final PidProperty m_topLeftPidProperty;
  private final SparkPIDController m_topRightPid;
  private final PidProperty m_topRightPidProperty;

  private final SparkPIDController m_bottomLeftPid;
  private final PidProperty m_bottomLeftPidProperty;
  private final SparkPIDController m_bottomRightPid;
  private final PidProperty m_bottomRightPidProperty;


  public ShooterIOPrototype() {
    m_topLeftMotor = new CANSparkFlex(ShooterConstants.TOP_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_topRightMotor = new CANSparkFlex(ShooterConstants.TOP_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomLeftMotor = new CANSparkFlex(ShooterConstants.BOTTOM_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomRightMotor = new CANSparkFlex(ShooterConstants.BOTTOM_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_kickerMotor = new CANSparkMax(ShooterConstants.KICKER_ID, CANSparkLowLevel.MotorType.kBrushless);

    m_topLeftMotor.restoreFactoryDefaults();
    m_topRightMotor.restoreFactoryDefaults();
    m_bottomLeftMotor.restoreFactoryDefaults();
    m_bottomRightMotor.restoreFactoryDefaults();

    m_topLeftMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    m_topRightMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    m_bottomLeftMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    m_bottomRightMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

    m_topLeftMotor.setInverted(ShooterConstants.TOP_LEFT_INVERTED);
    m_topRightMotor.setInverted(ShooterConstants.TOP_RIGHT_INVERTED);
    m_bottomLeftMotor.setInverted(ShooterConstants.BOTTOM_LEFT_INVERTED);
    m_bottomRightMotor.setInverted(ShooterConstants.BOTTOM_RIGHT_INVERTED);
    m_kickerMotor.setInverted(ShooterConstants.KICKER_INVERTED);

    m_topLeftMotor.enableVoltageCompensation(12);
    m_topRightMotor.enableVoltageCompensation(12);
    m_bottomLeftMotor.enableVoltageCompensation(12);
    m_bottomRightMotor.enableVoltageCompensation(12);

    m_topLeftPid = m_topLeftMotor.getPIDController();
    m_topLeftPidProperty = new RevPidPropertyBuilder("Shooter/Top Left Shooter", false, m_topLeftPid, 0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addI(ShooterConstants.SHOOTER_KI)
        .addD(ShooterConstants.SHOOTER_KD)
        .addFF(ShooterConstants.SHOOTER_KF)
        .build();

    m_topRightPid = m_topRightMotor.getPIDController();
    m_topRightPidProperty = new RevPidPropertyBuilder("Shooter/Top Right Shooter", false, m_topRightPid, 0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addI(ShooterConstants.SHOOTER_KI)
        .addD(ShooterConstants.SHOOTER_KD)
        .addFF(ShooterConstants.SHOOTER_KF)
        .build();

    m_bottomLeftPid = m_bottomLeftMotor.getPIDController();
    m_bottomLeftPidProperty = new RevPidPropertyBuilder("Shooter/Bottom Left Shooter", false, m_bottomLeftPid, 0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addI(ShooterConstants.SHOOTER_KI)
        .addD(ShooterConstants.SHOOTER_KD)
        .addFF(ShooterConstants.SHOOTER_KF)
        .build();

    m_bottomRightPid = m_bottomRightMotor.getPIDController();
    m_bottomRightPidProperty = new RevPidPropertyBuilder("Shooter/Bottom Right Shooter", false, m_bottomRightPid, 0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addI(ShooterConstants.SHOOTER_KI)
        .addD(ShooterConstants.SHOOTER_KD)
        .addFF(ShooterConstants.SHOOTER_KF)
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
  public void setLeftVelocityRpm(double rpm) {
    m_topLeftPid.setReference(rpm, CANSparkBase.ControlType.kVelocity);
    m_bottomLeftPid.setReference(rpm, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void setRightVelocityRpm(double rpm) {
    m_topRightPid.setReference(rpm, CANSparkBase.ControlType.kVelocity);
    m_bottomRightPid.setReference(rpm, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_topLeftPidProperty.updateIfChanged();
    m_topRightPidProperty.updateIfChanged();
    m_bottomLeftPidProperty.updateIfChanged();
    m_bottomRightPidProperty.updateIfChanged();

    inputs.setTlVelocityRPM(m_topLeftMotor.getEncoder().getVelocity() / 60.0);
    inputs.setTrVelocityRPM(m_topRightMotor.getEncoder().getVelocity() / 60.0);
    inputs.setBlVelocityRPM(m_bottomLeftMotor.getEncoder().getVelocity() / 60.0);
    inputs.setBrVelocityRPM(m_bottomRightMotor.getEncoder().getVelocity() / 60.0);

    inputs.setTlAppliedVolts(m_topLeftMotor.getAppliedOutput() * m_topLeftMotor.getBusVoltage());
    inputs.setTrAppliedVolts(m_topRightMotor.getAppliedOutput() * m_topRightMotor.getBusVoltage());
    inputs.setBlAppliedVolts(m_bottomLeftMotor.getAppliedOutput() * m_bottomLeftMotor.getBusVoltage());
    inputs.setBrAppliedVolts(m_bottomRightMotor.getAppliedOutput() * m_bottomRightMotor.getBusVoltage());
    inputs.setKickerAppliedVolts(m_kickerMotor.getAppliedOutput() * m_kickerMotor.getBusVoltage());

    inputs.setTlCurrentDraw(m_topLeftMotor.getOutputCurrent());
    inputs.setTrCurrentDraw(m_topRightMotor.getOutputCurrent());
    inputs.setBlCurrentDraw(m_bottomLeftMotor.getOutputCurrent());
    inputs.setBrCurrentDraw(m_bottomRightMotor.getOutputCurrent());
    inputs.setKickerCurrentDraw(m_kickerMotor.getOutputCurrent());

    inputs.setTlTemperature(m_topLeftMotor.getMotorTemperature());
    inputs.setTrTemperature(m_topRightMotor.getMotorTemperature());
    inputs.setBlTemperature(m_bottomLeftMotor.getMotorTemperature());
    inputs.setBrTemperature(m_bottomRightMotor.getMotorTemperature());
  }
}
