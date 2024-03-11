package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.ClimberConstants;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import lib.factories.TalonFXFactory;

public class ClimberIOPrototype implements ClimberIO {
  private final TalonFX m_leftTalon;
  private final TalonFX m_rightTalon;
  private final PidPropertyPublic m_leftClimberPid;
  private final PidPropertyPublic m_rightClimberPid;

  public ClimberIOPrototype() {


    var rightTalonConfig = new TalonFXConfiguration();
    rightTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_rightTalon = TalonFXFactory.createTalon(ClimberConstants.RIGHT_CLIMBER_ID, "canivore", rightTalonConfig);

    var leftTalonConfig = new TalonFXConfiguration();

    m_leftTalon = TalonFXFactory.createTalon(ClimberConstants.LEFT_CLIMBER_ID, "canivore", leftTalonConfig);

    m_leftClimberPid = new Phoenix6PidPropertyBuilder(
        "Climber/Left PID",
        false, m_leftTalon, 0)
        .addP(ClimberConstants.CLIMBER_KP)
        .addI(ClimberConstants.CLIMBER_KI)
        .addD(ClimberConstants.CLIMBER_KD)
        .build();

    m_rightClimberPid = new Phoenix6PidPropertyBuilder(
        "Climber/Right PID",
        false, m_rightTalon, 0)
        .addP(ClimberConstants.CLIMBER_KP)
        .addI(ClimberConstants.CLIMBER_KI)
        .addD(ClimberConstants.CLIMBER_KD)
        .build();

  }

  @Override
  public void setLeftVoltage(double volts) {
    m_leftTalon.setVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts) {
    m_rightTalon.setVoltage(volts);
  }

  @Override
  public void setLeftPosition(double degrees) {
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    m_leftTalon.setControl(request.withPosition(degrees / 360.0));
  }

  @Override
  public void setRightPosition(double degrees) {
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    m_rightTalon.setControl(request.withPosition(degrees / 360.0));
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    m_leftClimberPid.updateIfChanged();
    m_rightClimberPid.updateIfChanged();

    inputs.setLeftClimberPosition(m_leftTalon.getPosition().getValueAsDouble());
    inputs.setRightClimberPosition(m_rightTalon.getPosition().getValueAsDouble());

    inputs.setLeftClimberVelocity(m_leftTalon.getVelocity().getValueAsDouble());
    inputs.setRightClimberVelocity(m_rightTalon.getVelocity().getValueAsDouble());

    inputs.setLeftClimberCurrentDraw(m_leftTalon.getSupplyCurrent().getValueAsDouble());
    inputs.setRightClimberCurrentDraw(m_rightTalon.getSupplyCurrent().getValueAsDouble());

    inputs.setLeftClimberCurrentSetpoint(m_leftTalon.getClosedLoopReference().getValueAsDouble());
    inputs.setRightClimberCurrentSetpoint(m_rightTalon.getClosedLoopReference().getValueAsDouble());

    inputs.setLeftClimberAppliedOutput(m_leftTalon.getBridgeOutput().getValueAsDouble());
    inputs.setRightClimberAppliedOutput(m_rightTalon.getBridgeOutput().getValueAsDouble());
  }
}
