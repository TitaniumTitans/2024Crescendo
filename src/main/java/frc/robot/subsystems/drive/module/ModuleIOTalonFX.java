// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lib.factories.TalonFXFactory;
import lib.properties.phoenix6.Phoenix6PidPropertyBuilder;

import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX m_driveTalon;
  private final TalonFX m_turnTalon;
  private final CANcoder m_cancoder;

  // GoSProperties for motor controllers
  private final Phoenix6PidPropertyBuilder m_drivePid;
  private final Phoenix6PidPropertyBuilder m_turnPid;

  private final ModuleConstants m_moduleConstants;

  // Queues for odometry
  private final Queue<Double> m_drivePositionQueue;
  private final Queue<Double> m_turnPositionQueue;

  // status signals for drive motor
  private final StatusSignal<Double> m_drivePosition;
  private final StatusSignal<Double> m_driveVelocity;
  private final StatusSignal<Double> m_driveAppliedVolts;
  private final StatusSignal<Double> m_driveCurrent;

  // status signals for azimuth motor
  private final StatusSignal<Double> m_turnAbsolutePosition;
  private final StatusSignal<Double> m_turnPosition;
  private final StatusSignal<Double> m_turnVelocity;
  private final StatusSignal<Double> m_turnAppliedVolts;
  private final StatusSignal<Double> m_turnCurrent;
  private final StatusSignal<Double> m_turnClosedLoopOutput;
  private final StatusSignal<Double> m_turnClosedLoopError;
  private final StatusSignal<Double> m_turnClosedLoopReference;

  // control requests
  PositionVoltage m_posRequest;

  public ModuleIOTalonFX(ModuleConstants moduleConstants) {
    String canbus = "canivore";
    m_moduleConstants = moduleConstants;

    m_cancoder = new CANcoder(moduleConstants.ENCODER_ID(), canbus);

      // run factory default on cancoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection =
        moduleConstants.ENCODER_INVERTED() ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_cancoder.getConfigurator().apply(encoderConfig);

    m_turnAbsolutePosition = m_cancoder.getAbsolutePosition();

    // run configs on drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    driveConfig.MotorOutput.Inverted =
            moduleConstants.DRIVE_MOTOR_INVERTED() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = m_moduleConstants.DRIVE_GEAR_RATIO();

    m_driveTalon = TalonFXFactory.createTalon(moduleConstants.DRIVE_MOTOR_ID(), canbus, driveConfig);
    setDriveBrakeMode(true);

    // setup pid gains for drive motor
    m_drivePid = new Phoenix6PidPropertyBuilder(
            "Drive/Module" + m_moduleConstants.MODULE_INDEX() + "/Drive Pid Property",
            true, m_driveTalon, 0)
            .addP(m_moduleConstants.DRIVE_KP())
            .addI(m_moduleConstants.DRIVE_KI())
            .addD(m_moduleConstants.DRIVE_KD())
            .addKV(m_moduleConstants.DRIVE_KV())
            .addKS(m_moduleConstants.DRIVE_KS());

    // run configs on turning motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted =
            moduleConstants.TURN_MOTOR_INVERTED() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Feedback.SensorToMechanismRatio = m_moduleConstants.TURNING_GEAR_RATIO();

    m_turnTalon = TalonFXFactory.createTalon(moduleConstants.TURN_MOTOR_ID(), turnConfig);
    setTurnBrakeMode(true);

    // setup pid gains for turn motor
    m_turnPid = new Phoenix6PidPropertyBuilder(
            "Drive/Module" + m_moduleConstants.MODULE_INDEX() + "/Turn Pid Property",
            true,
            m_turnTalon,
            0)
            .addP(m_moduleConstants.TURN_KP())
            .addI(m_moduleConstants.TURN_KI())
            .addD(m_moduleConstants.TURN_KD());

    m_posRequest = new PositionVoltage(0, 0, false, 0, 0, false, false, false);

    // Fancy multithreaded odometry update stuff
    m_drivePosition = m_driveTalon.getPosition();
    m_turnPosition = m_turnTalon.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.DriveConstants.ODOMETRY_FREQUENCY, m_drivePosition, m_turnPosition);
    m_drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(m_driveTalon, m_drivePosition);
    m_turnPositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(m_turnTalon, m_turnPosition);

    // setup drive values
    m_driveTalon.setPosition(0.0);
    m_driveVelocity = m_driveTalon.getVelocity();
    m_driveAppliedVolts = m_driveTalon.getMotorVoltage();
    m_driveCurrent = m_driveTalon.getStatorCurrent();

    // setup turn values
    m_turnTalon.setPosition(m_turnAbsolutePosition.getValueAsDouble() - m_moduleConstants.ENCODER_OFFSET().getRotations());
    m_turnVelocity = m_turnTalon.getVelocity();
    m_turnAppliedVolts = m_turnTalon.getMotorVoltage();
    m_turnCurrent = m_turnTalon.getStatorCurrent();
    m_turnClosedLoopOutput = m_turnTalon.getClosedLoopOutput();
    m_turnClosedLoopError = m_turnTalon.getClosedLoopError();
    m_turnClosedLoopReference = m_turnTalon.getClosedLoopReference();

    // setup refresh rates on all inputs
    BaseStatusSignal.setUpdateFrequencyForAll(
        5.0,
        m_driveVelocity,
        m_driveAppliedVolts,
        m_driveCurrent,
        m_turnAbsolutePosition,
        m_turnVelocity,
        m_turnAppliedVolts,
        m_turnCurrent,
        m_turnClosedLoopOutput,
        m_turnClosedLoopError,
        m_turnClosedLoopReference);
    m_driveTalon.optimizeBusUtilization();
    m_turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    // update all signals
    BaseStatusSignal.refreshAll(
            m_drivePosition,
            m_driveVelocity,
            m_driveAppliedVolts,
            m_driveCurrent,
            m_turnAbsolutePosition,
            m_turnPosition,
            m_turnVelocity,
            m_turnAppliedVolts,
            m_turnCurrent,
            m_turnClosedLoopOutput,
            m_turnClosedLoopError,
            m_turnClosedLoopReference);

    m_drivePid.updateIfChanged();
    m_turnPid.updateIfChanged();

    // update the logged values of the drive motor
    inputs.drivePositionMeters = m_drivePosition.getValueAsDouble() * m_moduleConstants.WHEEL_CURCUMFERENCE_METERS();
    inputs.driveVelocityMPS = m_driveVelocity.getValueAsDouble() * m_moduleConstants.WHEEL_CURCUMFERENCE_METERS();
    inputs.driveAppliedVolts = m_driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {m_driveCurrent.getValueAsDouble()};

    // update the logged value of the encoder
    inputs.setTurnAbsolutePosition(Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble()));

    // update the logged values of the azimuth motor
    inputs.setTurnPosition(Rotation2d.fromRotations((m_turnPosition.getValueAsDouble())));
    inputs.setTurnVelocityRadPerSec(
            Units.rotationsToRadians(m_turnVelocity.getValueAsDouble()));
    inputs.setTurnAppliedVolts(m_turnAppliedVolts.getValueAsDouble());
    inputs.setTurnCurrentAmps(new double[] {m_turnCurrent.getValueAsDouble()});

    // update high speed odometry inputs
    inputs.odometryDrivePositionsMeters =
            m_drivePositionQueue.stream()
                    .mapToDouble(
                            signalValue -> signalValue * m_moduleConstants.WHEEL_CURCUMFERENCE_METERS())
                    .toArray();
    inputs.odometryTurnPositions =
            m_turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveVelocityMPS(double mps) {
    double rps = (mps / m_moduleConstants.WHEEL_CURCUMFERENCE_METERS()) * m_moduleConstants.DRIVE_GEAR_RATIO();
    VelocityVoltage velRequest = new VelocityVoltage(rps).withSlot(0).withEnableFOC(false);
    m_driveTalon.setControl(velRequest.withVelocity(rps));
  }

  @Override
  public void setTurnPositionRots(double rotations) {
    m_turnTalon.setControl(m_posRequest.withPosition(rotations));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
            m_moduleConstants.DRIVE_MOTOR_INVERTED() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
            m_moduleConstants.TURN_MOTOR_INVERTED() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_turnTalon.getConfigurator().apply(config);
  }

  @Override
  public ModuleConstants getModuleConstants() {
    return m_moduleConstants;
  }
}
