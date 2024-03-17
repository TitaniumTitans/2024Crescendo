package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimberConstants;
import lib.factories.TalonFXFactory;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;

public class ClimberIOKraken implements ClimberIO {
  private final TalonFX m_leftClimber;
  private final TalonFX m_rightClimber;

  private final PidPropertyPublic m_leftProperty;
  private final PidPropertyPublic m_rightProperty;

  private final PositionVoltage m_posRequest;

  private final StatusSignal<Double> m_leftPositionSignal;
  private final StatusSignal<Double> m_rightPositionSignal;
  private final StatusSignal<Double> m_leftVelocitySignal;
  private final StatusSignal<Double> m_rightVelocitySignal;
  private final StatusSignal<Double> m_leftCurrentDrawSignal;
  private final StatusSignal<Double> m_rightCurrentDrawSignal;
  private final StatusSignal<Double> m_leftSetpointSignal;
  private final StatusSignal<Double> m_rightSetpointSignal;
  private final StatusSignal<Double> m_leftAppliedOutputSignal;
  private final StatusSignal<Double> m_rightAppliedOutputSignal;

  public ClimberIOKraken() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_leftClimber = TalonFXFactory.createTalon(ClimberConstants.LEFT_CLIMBER_ID, climberConfig);

    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_rightClimber = TalonFXFactory.createTalon(ClimberConstants.RIGHT_CLIMBER_ID, climberConfig);

    m_leftProperty = new Phoenix6PidPropertyBuilder("Climber/Left Climber", false, m_leftClimber, 0)
        .addP(ClimberConstants.CLIMBER_KP)
        .addI(ClimberConstants.CLIMBER_KI)
        .addD(ClimberConstants.CLIMBER_KD)
        .addKS(ClimberConstants.CLIMBER_KS)
        .addKV(ClimberConstants.CLIMBER_KV)
        .addKG(ClimberConstants.CLIMBER_KG, GravityTypeValue.Elevator_Static)
        .build();

    m_rightProperty = new Phoenix6PidPropertyBuilder("Climber/Right Climber", false, m_rightClimber, 0)
        .addP(ClimberConstants.CLIMBER_KP)
        .addI(ClimberConstants.CLIMBER_KI)
        .addD(ClimberConstants.CLIMBER_KD)
        .addKS(ClimberConstants.CLIMBER_KS)
        .addKV(ClimberConstants.CLIMBER_KV)
        .addKG(ClimberConstants.CLIMBER_KG, GravityTypeValue.Elevator_Static)
        .build();

    m_posRequest = new PositionVoltage(0).withSlot(0)
        .withEnableFOC(m_leftClimber.getIsProLicensed().getValue() && m_rightClimber.getIsProLicensed().getValue());

    m_leftPositionSignal = m_leftClimber.getPosition();
    m_rightPositionSignal = m_rightClimber.getPosition();

    m_leftVelocitySignal = m_leftClimber.getVelocity();
    m_rightVelocitySignal = m_rightClimber.getVelocity();

    m_leftCurrentDrawSignal = m_leftClimber.getSupplyCurrent();
    m_rightCurrentDrawSignal = m_rightClimber.getSupplyCurrent();

    m_leftSetpointSignal = m_leftClimber.getClosedLoopReference();
    m_rightSetpointSignal = m_rightClimber.getClosedLoopReference();

    m_leftAppliedOutputSignal = m_leftClimber.getDutyCycle();
    m_rightAppliedOutputSignal = m_rightClimber.getDutyCycle();

    BaseStatusSignal.setUpdateFrequencyForAll(50,
        m_leftPositionSignal,
        m_rightPositionSignal,
        m_leftVelocitySignal,
        m_rightVelocitySignal,
        m_leftCurrentDrawSignal,
        m_rightCurrentDrawSignal,
        m_leftSetpointSignal,
        m_rightSetpointSignal,
        m_leftAppliedOutputSignal,
        m_rightAppliedOutputSignal);

    m_leftClimber.optimizeBusUtilization();
    m_rightClimber.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    inputs.leftClimberPosition = m_leftPositionSignal.getValueAsDouble();
    inputs.rightClimberPosition = m_rightPositionSignal.getValueAsDouble();

    inputs.leftClimberVelocity = m_leftVelocitySignal.getValueAsDouble();
    inputs.rightClimberVelocity = m_rightVelocitySignal.getValueAsDouble();

    inputs.leftClimberCurrentDraw = m_leftCurrentDrawSignal.getValueAsDouble();
    inputs.rightClimberCurrentDraw = m_rightCurrentDrawSignal.getValueAsDouble();

    inputs.leftClimberCurrentSetpoint = m_leftSetpointSignal.getValueAsDouble();
    inputs.rightClimberCurrentSetpoint = m_rightSetpointSignal.getValueAsDouble();

    inputs.leftClimberAppliedOutput = m_leftAppliedOutputSignal.getValueAsDouble();
    inputs.rightClimberAppliedOutput = m_rightAppliedOutputSignal.getValueAsDouble();

    m_leftProperty.updateIfChanged();
    m_rightProperty.updateIfChanged();
  }

  @Override
  public void setRightVoltage(double volts) {
    m_rightClimber.setVoltage(volts);
  }

  @Override
  public void setLeftVoltage(double volts) {
    m_leftClimber.setVoltage(volts);
  }

  @Override
  public void setRightPosition(double degrees) {
    m_rightClimber.setControl(m_posRequest.withPosition(degrees / 360));
  }

  @Override
  public void setLeftPosition(double degrees) {
    m_leftClimber.setControl(m_posRequest.withPosition(degrees / 360));
  }

  @Override
  public void stop() {
    m_leftClimber.setControl(new NeutralOut());
    m_rightClimber.setControl(new NeutralOut());
  }

  @Override
  public void resetPosition() {
    m_leftClimber.setPosition(0);
    m_rightClimber.setPosition(0);
  }
}
