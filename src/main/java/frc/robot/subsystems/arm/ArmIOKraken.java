package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import frc.robot.Constants.ArmConstants;

public class ArmIOKraken implements ArmIO {
  // Physical devices
  private final TalonFX m_armMaster;
  private final TalonFX m_armFollower;
  private final CANcoder m_armEncoder;

  private final TalonFX m_wristMaster;
  private final TalonFX m_wristFollower;
  private final CANcoder m_wristEncoder;

  // PID configs
  private final PidPropertyPublic m_armProperty;
  private final PidPropertyPublic m_wristProperty;

  // Control outputs
  private final PositionVoltage m_pidRequest;
  private final MotionMagicVoltage m_mmRequest;
  private final Follower m_armFollowerRequest;
  private final Follower m_wristFollowerRequest;
  private final NeutralOut m_stopRequest;

  // Status signals
  private final StatusSignal<Double> m_armPositionSignal;
  private final StatusSignal<Double> m_wristPositionSignal;
  private final StatusSignal<Double> m_armVelocitySignal;
  private final StatusSignal<Double> m_wristVelocitySignal;
  private final StatusSignal<Double> m_armOutputSignal;
  private final StatusSignal<Double> m_wristOutputSignal;
  private final StatusSignal<Double> m_armClosedOutputSignal;
  private final StatusSignal<Double> m_wristClosedOutputSignal;
  private final StatusSignal<Double> m_armSetpointSignal;
  private final StatusSignal<Double> m_wristSetpointSignal;
  private final StatusSignal<Double> m_armCurrentDrawSignal;
  private final StatusSignal<Double> m_wristCurrentDrawSignal;

  public ArmIOKraken() {
    final String CANBUS = "canivore";
    m_armMaster = new TalonFX(ArmConstants.SHOULDER_MASTER_ID, CANBUS);
    m_armFollower = new TalonFX(ArmConstants.SHOULDER_FOLLOWER_ID, CANBUS);
    m_armEncoder = new CANcoder(ArmConstants.SHOULDER_ENCODER_ID, CANBUS);

    m_wristMaster = new TalonFX(ArmConstants.WRIST_MASTER_ID, CANBUS);
    m_wristFollower = new TalonFX(ArmConstants.WRIST_FOLLOWER_ID, CANBUS);
    m_wristEncoder = new CANcoder(ArmConstants.WRIST_ENCODER_ID, CANBUS);

    // Arm Configuration
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
    armConfig.MotionMagic.MotionMagicAcceleration = 5;
    armConfig.CurrentLimits.SupplyCurrentLimit = 40;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    armConfig.Feedback.RotorToSensorRatio = 202.80;
    armConfig.Feedback.SensorToMechanismRatio = 1.0;

    m_armMaster.getConfigurator().apply(armConfig);
    m_armFollower.getConfigurator().apply(armConfig);

    // Wrist Configuration
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
    wristConfig.MotionMagic.MotionMagicAcceleration = 5;
    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristConfig.Feedback.RotorToSensorRatio = 152.10;
    wristConfig.Feedback.SensorToMechanismRatio = 1.0;

    m_wristMaster.getConfigurator().apply(wristConfig);
    m_wristFollower.getConfigurator().apply(wristConfig);

    // Encoder Configuration
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();
    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_armEncoder.getConfigurator().apply(armEncoderConfig);
    m_wristEncoder.getConfigurator().apply(wristEncoderConfig);

    // config pid
    m_armProperty = new Phoenix6PidPropertyBuilder("Arm/Arm PID", false, m_armMaster, 0)
        .addP(ArmConstants.SHOULDER_KP)
        .addI(ArmConstants.SHOULDER_KI)
        .addD(ArmConstants.SHOULDER_KD)
        .addKS(ArmConstants.SHOULDER_KS)
        .addKV(ArmConstants.SHOULDER_KV)
        .addKG(ArmConstants.SHOULDER_KG, GravityTypeValue.Arm_Cosine)
        .build();

    m_wristProperty = new Phoenix6PidPropertyBuilder("Arm/Wrist PID", false, m_wristMaster, 0)
        .addP(ArmConstants.WRIST_KP)
        .addI(ArmConstants.WRIST_KI)
        .addD(ArmConstants.WRIST_KD)
        .addKS(ArmConstants.WRIST_KS)
        .addKV(ArmConstants.WRIST_KV)
        .addKG(ArmConstants.WRIST_KG, GravityTypeValue.Arm_Cosine)
        .build();

    // config output requests
    m_pidRequest = new PositionVoltage(0)
        .withEnableFOC(m_armMaster.getIsProLicensed().getValue() && m_wristMaster.getIsProLicensed().getValue())
        .withSlot(0);
    m_mmRequest = new MotionMagicVoltage(0)
        .withEnableFOC(m_armMaster.getIsProLicensed().getValue() && m_wristMaster.getIsProLicensed().getValue())
        .withSlot(0);
    m_armFollowerRequest = new Follower(m_armMaster.getDeviceID(), false);
    m_wristFollowerRequest = new Follower(m_wristFollower.getDeviceID(), false);
    m_stopRequest = new NeutralOut();

    m_armPositionSignal = m_armEncoder.getAbsolutePosition();
    m_wristPositionSignal = m_wristEncoder.getAbsolutePosition();
    m_armVelocitySignal = m_armEncoder.getVelocity();
    m_wristVelocitySignal = m_wristEncoder.getVelocity();
    m_armOutputSignal = m_armMaster.getDutyCycle();
    m_wristOutputSignal = m_wristMaster.getDutyCycle();
    m_armClosedOutputSignal = m_armMaster.getClosedLoopOutput();
    m_wristClosedOutputSignal = m_wristMaster.getClosedLoopOutput();
    m_armSetpointSignal = m_armMaster.getClosedLoopReference();
    m_wristSetpointSignal = m_wristMaster.getClosedLoopReference();
    m_armCurrentDrawSignal = m_armMaster.getSupplyCurrent();
    m_wristCurrentDrawSignal = m_wristMaster.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_armPositionSignal,
        m_wristPositionSignal,
        m_armVelocitySignal,
        m_wristVelocitySignal,
        m_armOutputSignal,
        m_wristOutputSignal,
        m_armClosedOutputSignal,
        m_wristClosedOutputSignal,
        m_armSetpointSignal,
        m_wristSetpointSignal,
        m_armCurrentDrawSignal,
        m_wristCurrentDrawSignal);

    m_armMaster.optimizeBusUtilization();
    m_armFollower.optimizeBusUtilization();
    m_wristMaster.optimizeBusUtilization();
    m_wristFollower.optimizeBusUtilization();
    m_armEncoder.optimizeBusUtilization();
    m_wristEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    inputs.shoulderPositionRots = m_armPositionSignal.getValueAsDouble();
    inputs.wristPositionRots = m_wristPositionSignal.getValueAsDouble();

    inputs.shoulerVelocityRotsPerSecond = m_armVelocitySignal.getValueAsDouble();
    inputs.wristVelocityRotsPerSecond = m_wristVelocitySignal.getValueAsDouble();

    inputs.shoulderAppliedOutput = m_armOutputSignal.getValueAsDouble();
    inputs.wristAppliedOutput = m_wristOutputSignal.getValueAsDouble();

    inputs.shoulderClosedLoopOutput = m_armClosedOutputSignal.getValueAsDouble();
    inputs.wristClosedLoopOutput = m_wristClosedOutputSignal.getValueAsDouble();

    inputs.shoulderDesiredSetpoint = m_armSetpointSignal.getValueAsDouble();
    inputs.wristDesiredSetpoint = m_wristSetpointSignal.getValueAsDouble();

    inputs.shoulderCurrentDraw = m_armCurrentDrawSignal.getValueAsDouble();
    inputs.wristCurrentDraw = m_wristCurrentDrawSignal.getValueAsDouble();

    m_wristProperty.updateIfChanged();
    m_armProperty.updateIfChanged();
  }

  @Override
  public void setShoulderVoltage(double voltage) {
    m_armMaster.setVoltage(voltage);
    m_armFollower.setControl(m_armFollowerRequest);
  }

  @Override
  public void setShoulderAngle(double degrees, boolean useMM) {
    if (useMM) {
      m_armMaster.setControl(m_mmRequest.withPosition(degrees / 360));
    } else {
      m_armMaster.setControl(m_pidRequest.withPosition(degrees / 360));
    }
    m_armFollower.setControl(m_armFollowerRequest);
  }

  @Override
  public void setWristVoltage(double voltage) {
    m_wristMaster.setVoltage(voltage);
    m_wristFollower.setControl(m_wristFollowerRequest);
  }

  @Override
  public void setWristAngle(double degrees, boolean useMM) {
    if (useMM) {
      m_wristMaster.setControl(m_mmRequest.withPosition(degrees / 360));
    } else {
      m_wristMaster.setControl(m_pidRequest.withPosition(degrees / 360));
    }
    m_wristFollower.setControl(m_wristFollowerRequest);
  }

  @Override
  public void resetPosition() {
    m_wristEncoder.setPosition(0.0);
    m_armEncoder.setPosition(0.0);
  }

  @Override
  public void stop() {
    m_armMaster.setControl(m_stopRequest);
    m_wristMaster.setControl(m_stopRequest);
    m_armMaster.setControl(m_armFollowerRequest);
    m_wristMaster.setControl(m_wristFollowerRequest);
  }
}
