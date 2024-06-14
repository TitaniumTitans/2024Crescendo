package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import lib.factories.TalonFXFactory;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;
import lib.properties.phoenix6.PidPropertyPublic;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class ShooterIOKraken implements ShooterIO {
  private final TalonFX m_leftTalon;
  private final TalonFX m_rightTalon;
  private final TalonFX m_kicker;
  private final TalonFX m_intake;
  private final TalonFX m_indexer;

  private final TimeOfFlight m_tof;

  private final PidPropertyPublic m_leftProperty;
  private final PidPropertyPublic m_rightProperty;

  private final VelocityVoltage m_velRequest;
  private final NeutralOut m_stopRequest;

  private double[] m_prevIndexerCurrent = new double[] {0.0, 0.0, 0.0, 0.0};

  private final StatusSignal<Double> m_leftVelSignal;
  private final StatusSignal<Double> m_rightVelSignal;

  private final StatusSignal<Double> m_leftVoltOutSignal;
  private final StatusSignal<Double> m_rightVoltOutSignal;
  private final StatusSignal<Double> m_kickerVoltOutSignal;
  private final StatusSignal<Double> m_intakeVoltOutSignal;
  private final StatusSignal<Double> m_indexerVoltOutSignal;

  private final StatusSignal<Double> m_leftCurrentDrawSignal;
  private final StatusSignal<Double> m_rightCurrentDrawSignal;
  private final StatusSignal<Double> m_kickerCurrentDrawSignal;
  private final StatusSignal<Double> m_intakeCurrentDrawSignal;
  private final StatusSignal<Double> m_indexerCurrentDrawSignal;

  private final StatusSignal<Double> m_leftTemperatureSignal;
  private final StatusSignal<Double> m_rightTemperatureSignal;
  private final StatusSignal<Double> m_intakeTemperatureSignal;
  private final StatusSignal<Double> m_indexerTemperatureSignal;

  public ShooterIOKraken() {
    m_tof = new TimeOfFlight(28);
    m_tof.setRangingMode(TimeOfFlight.RangingMode.Short, 25);
   
    // general motor configs
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    shooterConfig.Feedback.SensorToMechanismRatio = 1;

    // right shooter isn't inverted
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_leftTalon = TalonFXFactory.createTalon(ShooterConstants.TOP_LEFT_ID, shooterConfig);

    // everything else is
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_rightTalon = TalonFXFactory.createTalon(ShooterConstants.TOP_RIGHT_ID, shooterConfig);

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_indexer = TalonFXFactory.createTalon(ShooterConstants.INDEXER_ID, shooterConfig);
    m_intake = TalonFXFactory.createTalon(ShooterConstants.INTAKE_ID, "rio", shooterConfig);
    m_kicker = TalonFXFactory.createTalon(ShooterConstants.KICKER_ID, shooterConfig);

    m_leftProperty = new Phoenix6PidPropertyBuilder("Shooter/Left PID", false, m_leftTalon, 0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addD(ShooterConstants.SHOOTER_KD)
        .addKS(ShooterConstants.SHOOTER_KS)
        .addKV(ShooterConstants.SHOOTER_KV)
        .build();

    m_rightProperty = new Phoenix6PidPropertyBuilder("Shooter/Right PID", false, m_rightTalon, 0)
        .addP(ShooterConstants.SHOOTER_KP)
        .addD(ShooterConstants.SHOOTER_KD)
        .addKS(ShooterConstants.SHOOTER_KS)
        .addKV(ShooterConstants.SHOOTER_KV)
        .build();

    m_velRequest = new VelocityVoltage(0)
        .withEnableFOC(false)
        .withSlot(0);
    m_stopRequest = new NeutralOut();

    // status signals
    m_leftVelSignal = m_leftTalon.getVelocity();
    m_rightVelSignal = m_rightTalon.getVelocity();

    m_leftVoltOutSignal = m_leftTalon.getMotorVoltage();
    m_rightVoltOutSignal = m_rightTalon.getMotorVoltage();
    m_kickerVoltOutSignal = m_kicker.getMotorVoltage();
    m_intakeVoltOutSignal = m_intake.getMotorVoltage();
    m_indexerVoltOutSignal = m_indexer.getMotorVoltage();

    m_leftCurrentDrawSignal = m_leftTalon.getSupplyCurrent();
    m_rightCurrentDrawSignal = m_rightTalon.getSupplyCurrent();
    m_kickerCurrentDrawSignal = m_kicker.getSupplyCurrent();
    m_intakeCurrentDrawSignal = m_intake.getSupplyCurrent();
    m_indexerCurrentDrawSignal = m_indexer.getSupplyCurrent();

    m_leftTemperatureSignal = m_leftTalon.getDeviceTemp();
    m_rightTemperatureSignal = m_rightTalon.getDeviceTemp();
    m_intakeTemperatureSignal = m_intake.getDeviceTemp();
    m_indexerTemperatureSignal = m_indexer.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        m_leftVelSignal,
        m_rightVelSignal,
        m_leftVoltOutSignal,
        m_rightVoltOutSignal,
        m_kickerVoltOutSignal,
        m_intakeVoltOutSignal,
        m_indexerVoltOutSignal,
        m_leftCurrentDrawSignal,
        m_rightCurrentDrawSignal,
        m_kickerCurrentDrawSignal,
        m_intakeCurrentDrawSignal,
        m_indexerCurrentDrawSignal,
        m_leftTemperatureSignal,
        m_rightTemperatureSignal,
        m_intakeTemperatureSignal,
        m_indexerTemperatureSignal);

    m_leftTalon.optimizeBusUtilization();
    m_rightTalon.optimizeBusUtilization();
    m_kicker.optimizeBusUtilization();
    m_intake.optimizeBusUtilization();
    m_indexer.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        m_leftVelSignal,
        m_rightVelSignal,
        m_leftVoltOutSignal,
        m_rightVoltOutSignal,
        m_kickerVoltOutSignal,
        m_intakeVoltOutSignal,
        m_indexerVoltOutSignal,
        m_leftCurrentDrawSignal,
        m_rightCurrentDrawSignal,
        m_kickerCurrentDrawSignal,
        m_intakeCurrentDrawSignal,
        m_indexerCurrentDrawSignal,
        m_leftTemperatureSignal,
        m_rightTemperatureSignal,
        m_intakeTemperatureSignal,
        m_indexerTemperatureSignal
    );

    inputs.tlVelocityRPM = m_leftVelSignal.refresh().getValueAsDouble() * 60.0;
    inputs.trVelocityRPM = m_rightVelSignal.refresh().getValueAsDouble() * 60.0;

    Logger.recordOutput("Shooter/SysId/Left Vel RPS", m_leftVelSignal.refresh().getValueAsDouble());
    Logger.recordOutput("Shooter/SysId/Right Vel RPS", m_rightVelSignal.refresh().getValueAsDouble());

    inputs.tlAppliedVolts = m_leftVoltOutSignal.getValueAsDouble();
    inputs.trAppliedVolts = m_rightVoltOutSignal.getValueAsDouble();
    inputs.kickerAppliedVolts = m_kickerVoltOutSignal.getValueAsDouble();
    inputs.intakeAppliedVolts = m_intakeVoltOutSignal.getValueAsDouble();
    inputs.indexerAppliedVolts = m_indexerVoltOutSignal.getValueAsDouble();

    inputs.tlCurrentDraw = m_leftCurrentDrawSignal.getValueAsDouble();
    inputs.trCurrentDraw = m_rightCurrentDrawSignal.getValueAsDouble();
    inputs.kickerCurrentDraw = m_kickerCurrentDrawSignal.getValueAsDouble();
    inputs.intakeCurrentDraw = m_intakeCurrentDrawSignal.getValueAsDouble();
    inputs.indexerCurrentDraw = m_indexerCurrentDrawSignal.getValueAsDouble();
    inputs.kickerCurrentDraw = m_kickerCurrentDrawSignal.getValueAsDouble();

    m_prevIndexerCurrent = new double[] {
        m_prevIndexerCurrent[1],
        m_prevIndexerCurrent[2],
        m_prevIndexerCurrent[3],
        inputs.indexerCurrentDraw
    };

    inputs.tlTemperature = m_leftTemperatureSignal.getValueAsDouble();
    inputs.trTemperature = m_rightTemperatureSignal.getValueAsDouble();
    inputs.intakeTemperature = m_intakeTemperatureSignal.getValueAsDouble();
    inputs.indexerTemperature = m_indexerTemperatureSignal.getValueAsDouble();

    inputs.tofDistanceIn = m_tof.getRange();

    m_leftProperty.updateIfChanged();
    m_rightProperty.updateIfChanged();
  }

  @Override
  public boolean hasPiece() {
    return m_tof.getRange() < 60.0;
  }

  @Override
  public void setMotorVoltageTL(double voltage) {
    m_leftTalon.setVoltage(voltage);
  }

  @Override
  public void setMotorVoltageTR(double voltage) {
    m_rightTalon.setVoltage(voltage);
  }

  @Override
  public void setKickerVoltage(double voltage) {
    m_kicker.setVoltage(voltage);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    m_intake.setVoltage(voltage);
    if (Arrays.stream(m_prevIndexerCurrent).sum() / 5.0 > 40.0) {
      m_indexer.setVoltage(voltage * 0.5);
    } else {
      m_indexer.setVoltage(voltage);
    }
  }

  @Override
  public void setLeftVelocityRpm(double rpm) {
    m_leftTalon.setControl(m_velRequest.withVelocity(rpm / 60));
  }

  @Override
  public void setRightVelocityRpm(double rpm) {
    m_rightTalon.setControl(m_velRequest.withVelocity(rpm / 60));
  }

  @Override
  public void stop() {
    m_leftTalon.setControl(m_stopRequest);
    m_rightTalon.setControl(m_stopRequest);
    m_kicker.setControl(m_stopRequest);
    m_intake.setControl(m_stopRequest);
    m_indexer.setControl(m_stopRequest);
  }
}
