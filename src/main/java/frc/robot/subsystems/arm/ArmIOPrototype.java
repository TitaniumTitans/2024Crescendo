package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ArmConstants;

public class ArmIOPrototype implements ArmIO {
  private final TalonFX m_shoulder;
  private final TalonFX m_wrist;
  private final PidPropertyPublic m_shoulderPID;
  private final PidPropertyPublic m_wristPID;

  private final PositionVoltage m_wristReqPID;
  private final PositionVoltage m_shoulderReqPID;

  private final MotionMagicVoltage m_wristReqMM;
  private final MotionMagicVoltage m_shoulderReqMM;

  public ArmIOPrototype() {
    m_shoulder = new TalonFX(23);
    m_wrist = new TalonFX(24);

    TalonFXConfiguration config = new TalonFXConfiguration();
    // config for the shoulder joint
    config.Feedback.SensorToMechanismRatio = 125.0 * (60.0 / 18.0);

    config.CurrentLimits.
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;

    // motion magic configs for both joints
    config.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(360);
    config.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(720);

    m_shoulder.getConfigurator().apply(config);

    // wrist joint config
    config.Feedback.SensorToMechanismRatio = 125.0 * (38.0 / 18.0);
    m_wrist.getConfigurator().apply(config);

    m_wrist.stopMotor();
    m_shoulder.stopMotor();

    m_shoulderPID = new Phoenix6PidPropertyBuilder(
        "Arm/Shoulder PID",
        false,
        m_shoulder,
        0)
        .addP(ArmConstants.SHOULDER_KP)
        .addI(ArmConstants.SHOULDER_KI)
        .addD(ArmConstants.SHOULDER_KD)
        .addKS(ArmConstants.SHOULDER_KS)
        .addKV(ArmConstants.SHOULDER_KV)
        .addKG(ArmConstants.SHOULDER_KG, GravityTypeValue.Arm_Cosine)
        .build();

    m_wristPID = new Phoenix6PidPropertyBuilder(
        "Arm/Wrist PID",
        false,
        m_wrist,
        0)
        .addP(ArmConstants.WRIST_KP)
        .addI(ArmConstants.WRIST_KI)
        .addD(ArmConstants.WRIST_KD)
        .addKS(ArmConstants.WRIST_KS)
        .addKV(ArmConstants.WRIST_KV)
        .addKG(ArmConstants.WRIST_KG, GravityTypeValue.Arm_Cosine)
        .build();

    m_shoulderReqPID = new PositionVoltage(0).withSlot(0);
    m_wristReqPID = new PositionVoltage(0).withSlot(0);
    m_shoulderReqMM = new MotionMagicVoltage(0).withSlot(0);
    m_wristReqMM = new MotionMagicVoltage(0).withSlot(0);
  }

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    m_shoulderPID.updateIfChanged();
    m_wristPID.updateIfChanged();

    inputs.shoulderPositionRots = m_shoulder.getPosition().getValueAsDouble();
    inputs.wristPositionRots = m_wrist.getPosition().getValueAsDouble();

    inputs.shoulerVelocityRotsPerSecond = m_shoulder.getVelocity().getValueAsDouble();
    inputs.wristVelocityRotsPerSecond = m_wrist.getVelocity().getValueAsDouble();

    inputs.shoulderAppliedOutput = m_shoulder.get();
    inputs.wristAppliedOutput = m_wrist.get();

    inputs.shoulderClosedLoopOutput = m_shoulder.getClosedLoopOutput().getValueAsDouble();
    inputs.wristClosedLoopOutput = m_wrist.getClosedLoopOutput().getValueAsDouble();

    inputs.shoulderDesiredSetpoint = m_shoulder.getClosedLoopReference().getValueAsDouble();
    inputs.wristDesiredSetpoint = m_wrist.getClosedLoopReference().getValueAsDouble();

    inputs.shoulderCurrentDraw = m_shoulder.getSupplyCurrent().getValueAsDouble();
    inputs.wristCurrentDraw = m_wrist.getSupplyCurrent().getValueAsDouble();

    Logger.recordOutput("Arm/Should PID Output", m_shoulder.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput("Arm/Wrist PID Output", m_wrist.getClosedLoopOutput().getValueAsDouble());

    Logger.recordOutput("Arm/Shoulder PID Setpoint", m_shoulder.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput("Arm/Wrist PID Setpoint", m_wrist.getClosedLoopReference().getValueAsDouble());
  }

  @Override
  public void setShoulderVoltage(double voltage){
    m_shoulder.setVoltage(voltage);
  }

  @Override
  public void setShoulderAngle(double degrees, boolean useMM) {
    if (useMM) {
      m_shoulder.setControl(m_shoulderReqMM.withPosition(degrees / 360.0));
    } else {
      m_shoulder.setControl(m_shoulderReqPID.withPosition(degrees / 360.0));
    }
  }

  @Override
  public void setWristVoltage(double voltage){
    m_wrist.setVoltage(voltage);
  }

  @Override
  public void setWristAngle(double degrees, boolean useMM) {
    if (useMM) {
      m_wrist.setControl(m_wristReqMM.withPosition(degrees / 360.0));
    } else {
      m_wrist.setControl(m_wristReqPID.withPosition(degrees / 360.0));
    }
  }
}
