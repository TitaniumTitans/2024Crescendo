package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.HeavyDoubleProperty;
import edu.wpi.first.math.util.Units;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

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

  private final HeavyDoubleProperty m_armMaxVelDegS;
  private final HeavyDoubleProperty m_wristMaxVelDegS;
  private final HeavyDoubleProperty m_accelTimeSecs;

  private final MotionMagicConfigs m_armMMConfigs = new MotionMagicConfigs();
  private final MotionMagicConfigs m_wristMMConfigs = new MotionMagicConfigs();

  // Control outputs
  private final DynamicMotionMagicVoltage m_wristDynMMRequest;
  private final DynamicMotionMagicVoltage m_armDynMMRequest;
  private final Follower m_armFollowerRequest;
  private final Follower m_wristFollowerRequest;
  private final NeutralOut m_stopRequest;

  private boolean m_tracking = false;

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
    m_armMaster = new TalonFX(ArmConstants.ARM_MASTER_ID, CANBUS);
    m_armFollower = new TalonFX(ArmConstants.ARM_FOLLOWER_ID, CANBUS);
    m_armEncoder = new CANcoder(ArmConstants.ARM_ENCODER_ID, CANBUS);

    m_wristMaster = new TalonFX(ArmConstants.WRIST_MASTER_ID, CANBUS);
    m_wristFollower = new TalonFX(ArmConstants.WRIST_FOLLOWER_ID, CANBUS);
    m_wristEncoder = new CANcoder(ArmConstants.WRIST_ENCODER_ID, CANBUS);

    // Arm Configuration
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.CurrentLimits.SupplyCurrentLimit = 40;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    armConfig.Feedback.SensorToMechanismRatio = ArmConstants.ARM_SENSOR_MECHANISM_RATIO;

    m_armMaster.getConfigurator().apply(armConfig);
    m_armFollower.getConfigurator().apply(armConfig);

    // Wrist Configuration
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.Feedback.SensorToMechanismRatio = ArmConstants.WRIST_SENSOR_MECHANISM_RATIO;

    m_wristMaster.getConfigurator().apply(wristConfig);
    m_wristFollower.getConfigurator().apply(wristConfig);

    // Encoder Configuration
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = ArmConstants.ARM_OFFSET;

    CANcoderConfiguration wristEncoderConfig = new CANcoderConfiguration();
    wristEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristEncoderConfig.MagnetSensor.MagnetOffset = ArmConstants.WRIST_OFFSET;

    m_armEncoder.getConfigurator().apply(armEncoderConfig);
    m_wristEncoder.getConfigurator().apply(wristEncoderConfig);

    // config output requests
    m_wristDynMMRequest = new DynamicMotionMagicVoltage(0, 0, 0, 0)
        .withEnableFOC(m_armMaster.getIsProLicensed().getValue() && m_wristMaster.getIsProLicensed().getValue())
        .withSlot(0);

    m_armDynMMRequest = new DynamicMotionMagicVoltage(0, 0, 0, 0)
        .withEnableFOC(m_armMaster.getIsProLicensed().getValue() && m_wristMaster.getIsProLicensed().getValue())
        .withSlot(0);

    // config pid and properties
    m_armProperty = new Phoenix6PidPropertyBuilder("Arm/Arm PID", false, m_armMaster, 0)
        .addP(ArmConstants.ARM_KP)
        .addI(ArmConstants.ARM_KI)
        .addD(ArmConstants.ARM_KD)
        .addKS(ArmConstants.ARM_KS)
        .addKV(ArmConstants.ARM_KV)
        .addKG(ArmConstants.ARM_KG, GravityTypeValue.Arm_Cosine)
        .build();

    m_wristProperty = new Phoenix6PidPropertyBuilder("Arm/Wrist PID", false, m_wristMaster, 0)
        .addP(ArmConstants.WRIST_KP)
        .addI(ArmConstants.WRIST_KI)
        .addD(ArmConstants.WRIST_KD)
        .addKS(ArmConstants.WRIST_KS)
        .addKV(ArmConstants.WRIST_KV)
        .addKG(ArmConstants.WRIST_KG, GravityTypeValue.Arm_Cosine)
        .build();

    m_armMaxVelDegS = new HeavyDoubleProperty(
        (double maxVel) -> m_armDynMMRequest.Velocity = Units.degreesToRotations(maxVel),
        new GosDoubleProperty(false, "Arm/Arm Max Vel DegsS", 120));

    m_wristMaxVelDegS = new HeavyDoubleProperty(
        (double maxVel) -> m_wristDynMMRequest.Velocity = Units.degreesToRotations(maxVel),
        new GosDoubleProperty(false, "Arm/Wrist Max Vel DegsS", 120));

    m_accelTimeSecs = new HeavyDoubleProperty((double accel) -> {
      m_armDynMMRequest.Acceleration = m_armDynMMRequest.Velocity / accel;
      m_wristDynMMRequest.Acceleration = m_wristDynMMRequest.Velocity / accel;
    }, new GosDoubleProperty(false, "Arm/Acceleration Time Secs", 1));

    m_armMaxVelDegS.updateIfChanged(true);
    m_wristMaxVelDegS.updateIfChanged(true);
    m_accelTimeSecs.updateIfChanged(true);

    m_armFollowerRequest = new Follower(m_armMaster.getDeviceID(), false);
    m_wristFollowerRequest = new Follower(m_wristMaster.getDeviceID(), false);
    m_stopRequest = new NeutralOut();

    m_armPositionSignal = m_armMaster.getPosition();
    m_wristPositionSignal = m_wristMaster.getPosition();
    m_armVelocitySignal = m_armMaster.getVelocity();
    m_wristVelocitySignal = m_wristMaster.getVelocity();
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
        m_wristCurrentDrawSignal,
        m_armEncoder.getAbsolutePosition(),
        m_wristEncoder.getAbsolutePosition());

    m_armMaster.optimizeBusUtilization();
    m_armFollower.optimizeBusUtilization();
    m_wristMaster.optimizeBusUtilization();
    m_wristFollower.optimizeBusUtilization();
    m_armEncoder.optimizeBusUtilization();
    m_wristEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
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
            m_wristCurrentDrawSignal
    );

    inputs.armPositionDegs = Units.rotationsToDegrees(m_armPositionSignal.getValueAsDouble());
    inputs.wristPositionDegs = Units.rotationsToDegrees(m_wristPositionSignal.getValueAsDouble());

    inputs.armVelocityDegsPerSecond = Units.rotationsToDegrees(m_armVelocitySignal.getValueAsDouble());
    inputs.wristVelocityDegsPerSecond = Units.rotationsToDegrees(m_wristVelocitySignal.getValueAsDouble());

    inputs.armAppliedOutput = m_armOutputSignal.getValueAsDouble();
    inputs.wristAppliedOutput = m_wristOutputSignal.getValueAsDouble();

    inputs.armClosedLoopOutput = m_armClosedOutputSignal.getValueAsDouble();
    inputs.wristClosedLoopOutput = m_wristClosedOutputSignal.getValueAsDouble();

    inputs.armDesiredSetpoint = Units.rotationsToDegrees(m_armSetpointSignal.getValueAsDouble())
        / ArmConstants.ARM_SENSOR_MECHANISM_RATIO;
    inputs.wristDesiredSetpoint = Units.rotationsToDegrees(m_wristSetpointSignal.getValueAsDouble())
        / ArmConstants.WRIST_SENSOR_MECHANISM_RATIO;

    inputs.armCurrentDraw = m_armCurrentDrawSignal.getValueAsDouble();
    inputs.wristCurrentDraw = m_wristCurrentDrawSignal.getValueAsDouble();

    Logger.recordOutput("Arm Absolute Position", Units.rotationsToDegrees(
        m_armEncoder.getAbsolutePosition().getValueAsDouble()));
    Logger.recordOutput("Wrist Absolute Position", Units.rotationsToDegrees(
        m_wristEncoder.getAbsolutePosition().getValueAsDouble()));

    // update any pid properties
    m_wristProperty.updateIfChanged();
    m_armProperty.updateIfChanged();

    m_armMaxVelDegS.updateIfChanged();
    m_wristMaxVelDegS.updateIfChanged();
    m_accelTimeSecs.updateIfChanged();
  }

  @Override
  public void setArmVoltage(double voltage) {
    m_armMaster.setVoltage(voltage);
    m_armFollower.setControl(m_armFollowerRequest);
  }

  @Override
  public void setArmAngle(double degrees, double velocityMult) {
    m_armMaxVelDegS.updateIfChanged(true);
    m_armDynMMRequest.Velocity = m_armDynMMRequest.Velocity * velocityMult;

    m_armMaster.setControl(m_armDynMMRequest.withPosition(degrees / 360));
    m_armFollower.setControl(m_armFollowerRequest);
  }

  @Override
  public void setWristVoltage(double voltage) {
    m_wristFollower.setControl(m_wristFollowerRequest);
    m_wristMaster.setVoltage(voltage);
  }

  @Override
  public void setWristAngle(double degrees, double velocityMult) {
    m_wristMaxVelDegS.updateIfChanged(true);
    m_wristDynMMRequest.Velocity = m_wristDynMMRequest.Velocity * velocityMult;

    m_wristMaster.setControl(m_wristDynMMRequest.withPosition(degrees / 360));
    m_wristFollower.setControl(m_wristFollowerRequest);
  }

  // We have to nudge our "zero" value because of the gear ratio
  @Override
  public void resetPosition() {
    m_armMaster.setPosition(
        (m_armEncoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(ArmConstants.OFFSET_NUDGE))
            / ArmConstants.ARM_CANCODER_MECHANISM_RATIO);

    m_wristMaster.setPosition(
        (m_wristEncoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(ArmConstants.OFFSET_NUDGE))
            / ArmConstants.WRIST_CANCODER_MECHANISM_RATIO);
  }

  @Override
  public void stop() {
    m_armMaster.setControl(m_stopRequest);
    m_wristMaster.setControl(m_stopRequest);
    m_armMaster.setControl(m_armFollowerRequest);
    m_wristMaster.setControl(m_wristFollowerRequest);
  }

  @Override
  public void enableBrakeMode(boolean enabled) {
    MotorOutputConfigs config = new MotorOutputConfigs();

    config.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Inverted = InvertedValue.Clockwise_Positive;

    m_wristMaster.getConfigurator().apply(config);
    m_wristFollower.getConfigurator().apply(config);

    m_armMaster.getConfigurator().apply(config);
    m_armFollower.getConfigurator().apply(config);
  }
}
