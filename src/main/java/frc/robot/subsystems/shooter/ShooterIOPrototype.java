package frc.robot.subsystems.shooter;

import com.gos.lib.properties.feedforward.SimpleMotorFeedForwardProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOPrototype implements ShooterIO {
  private final CANSparkFlex m_topLeftMotor;
  private final CANSparkFlex m_topRightMotor;
  private final CANSparkMax m_bottomLeftMotor;
  private final CANSparkMax m_bottomRightMotor;
  private final CANSparkMax m_kickerMotor;
  private final CANSparkMax m_intakeLeft;
  private final CANSparkMax m_intakeRight;

  private final SparkPIDController m_tlController;
  private final PidProperty m_tlPid;
  private final SparkPIDController m_trController;
  private final PidProperty m_trPid;
  private final SparkPIDController m_blController;
  private final PidProperty m_blPid;
  private final SparkPIDController m_brController;
  private final PidProperty m_brPid;

  private final SimpleMotorFeedforward m_shooterFF;
  private final SimpleMotorFeedForwardProperty m_shooterFFProperty;

  public ShooterIOPrototype() {
    m_topLeftMotor = new CANSparkFlex(ShooterConstants.TL_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_topRightMotor = new CANSparkFlex(ShooterConstants.TR_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomLeftMotor = new CANSparkMax(ShooterConstants.BL_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomRightMotor = new CANSparkMax(ShooterConstants.BR_SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_kickerMotor = new CANSparkMax(ShooterConstants.KICKER_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeLeft = new CANSparkMax(ShooterConstants.INTAKE_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeRight = new CANSparkMax(ShooterConstants.INTAKE_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);

    m_topLeftMotor.setInverted(false);
    m_bottomLeftMotor.setInverted(false);

    m_intakeRight.setInverted(true);
    m_intakeLeft.setInverted(true);
    m_kickerMotor.setInverted(true);

    m_tlController = m_topLeftMotor.getPIDController();
    m_tlPid = new RevPidPropertyBuilder("Shooter/TL PID", false, m_tlController, 0)
            .addP(ShooterConstants.SHOOTER_KP)
            .addI(ShooterConstants.SHOOTER_KI)
            .addD(ShooterConstants.SHOOTER_KD)
            .build();

    m_trController = m_topLeftMotor.getPIDController();
    m_trPid = new RevPidPropertyBuilder("Shooter/TR PID", false, m_trController, 0)
            .addP(ShooterConstants.SHOOTER_KP)
            .addI(ShooterConstants.SHOOTER_KI)
            .addD(ShooterConstants.SHOOTER_KD)
            .build();

    m_blController = m_topLeftMotor.getPIDController();
    m_blPid = new RevPidPropertyBuilder("Shooter/BL PID", false, m_blController, 0)
            .addP(ShooterConstants.SHOOTER_KP)
            .addI(ShooterConstants.SHOOTER_KI)
            .addD(ShooterConstants.SHOOTER_KD)
            .build();

    m_brController = m_topLeftMotor.getPIDController();
    m_brPid = new RevPidPropertyBuilder("Shooter/BR PID", false, m_brController, 0)
            .addP(ShooterConstants.SHOOTER_KP)
            .addI(ShooterConstants.SHOOTER_KI)
            .addD(ShooterConstants.SHOOTER_KD)
            .build();

    m_shooterFF = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV);
    m_shooterFFProperty = new SimpleMotorFeedForwardProperty("Shooter FF", false, m_shooterFF);

    m_topLeftMotor.burnFlash();
    m_topRightMotor.burnFlash();
    m_intakeLeft.burnFlash();
    m_intakeRight.burnFlash();
    m_kickerMotor.burnFlash();
  }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.tLAngularVelocity = m_topLeftMotor.getEncoder().getVelocity() * Math.PI;
        inputs.tRAngularVelocity = m_topRightMotor.getEncoder().getVelocity() * Math.PI;
        inputs.bLAngularVelocity = m_bottomLeftMotor.getEncoder().getVelocity() * Math.PI;
        inputs.bRAngularVelocity = m_bottomRightMotor.getEncoder().getVelocity() * Math.PI;
        inputs.kickerAngularVelocity = m_kickerMotor.getEncoder().getVelocity() * Math.PI;

        inputs.tLAppliedInputs = m_topLeftMotor.getAppliedOutput();
        inputs.tRAppliedInputs = m_topRightMotor.getAppliedOutput();
        inputs.bLAppliedInputs = m_bottomLeftMotor.getAppliedOutput();
        inputs.bRAppliedInputs = m_bottomRightMotor.getAppliedOutput();
        inputs.kickerAppliedInputs = m_kickerMotor.getAppliedOutput();
    }

  @Override
  public void setVelocityTL(double rpm) {
    m_tlController.setReference(rpm,
            CANSparkBase.ControlType.kVelocity,
            0,
            m_shooterFFProperty.calculate(rpm),
            SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVelocityTR(double rpm) {
    m_topRightMotor.setVoltage(rpm);
  }

  @Override
  public void setVelocityBL(double rpm) {
    m_bottomLeftMotor.setVoltage(rpm);
  }

  @Override
  public void setVelocityBR(double rpm) {
    m_bottomRightMotor.setVoltage(rpm);
  }

  @Override
  public void setKickerVoltage(double voltage) {
    m_kickerMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    m_intakeLeft.setVoltage(voltage);
    m_intakeRight.setVoltage(voltage);
  }
}
