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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.gos.lib.phoenix6.properties.pid.Phoenix6TalonPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

  private final PidProperty m_drivePid;
  private final PidProperty m_turnPid;

  private final ModuleConstants m_moduleConstants;

  private final StatusSignal<Double> m_drivePosition;
  private final Queue<Double> m_drivePositionQueue;
  private final StatusSignal<Double> m_driveVelocity;
  private final StatusSignal<Double> m_driveAppliedVolts;
  private final StatusSignal<Double> m_driveCurrent;

  private final StatusSignal<Double> m_turnAbsolutePosition;
  private final StatusSignal<Double> m_turnPosition;
  private final Queue<Double> m_turnPositionQueue;
  private final StatusSignal<Double> m_turnVelocity;
  private final StatusSignal<Double> m_turnAppliedVolts;
  private final StatusSignal<Double> m_turnCurrent;

  public ModuleIOTalonFX(ModuleConstants moduleConstants) {
    m_driveTalon = new TalonFX(moduleConstants.DRIVE_MOTOR_ID());
    m_turnTalon = new TalonFX(moduleConstants.TURN_MOTOR_ID());
    /*
    * this is technically the proper way of using any class that
    * implements the "Closeable" or "AutoClosable", typically things
    * like files or network ports, but also robot hardware */
    try (CANcoder cancoder = new CANcoder(moduleConstants.ENCODER_ID())) {
      // run factory default on cancoder
      var encoderConfig = new CANcoderConfiguration();
      encoderConfig.MagnetSensor.SensorDirection =
              moduleConstants.ENCODER_INVERTED() ? SensorDirectionValue.Clockwise_Positive
                      : SensorDirectionValue.CounterClockwise_Positive;
      encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
      cancoder.getConfigurator().apply(encoderConfig);

      m_turnAbsolutePosition = cancoder.getAbsolutePosition();
    }

    m_moduleConstants = moduleConstants;

    // run configs on drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
            moduleConstants.DRIVE_MOTOR_INVERTED() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
    m_driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // setup pid gains for drive motor
    m_drivePid = new Phoenix6TalonPidPropertyBuilder(
            "Drive/Module" + m_moduleConstants.MODULE_INDEX() + "/Drive Pid Property",
            false,
            m_driveTalon,
            0
    ).addP(m_moduleConstants.DRIVE_KP()).build();

    // run configs on turning motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted =
            moduleConstants.TURN_MOTOR_INVERTED() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
    m_turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    m_turnPid = new Phoenix6TalonPidPropertyBuilder(
            "Drive/Module" + m_moduleConstants.MODULE_INDEX() + "/Turn Pid Property",
            false,
            m_turnTalon,
            0)
            .addP(m_moduleConstants.TURN_KP())
            .addI(m_moduleConstants.TURN_KI())
            .addD(m_moduleConstants.TURN_KD())
            .build();

    // Fancy multithreaded odometry update stuff
    // setup drive values
    m_driveTalon.setPosition(0.0);
    m_drivePosition = m_driveTalon.getPosition();
    m_drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(m_driveTalon, m_driveTalon.getPosition());
    m_driveVelocity = m_driveTalon.getVelocity();
    m_driveAppliedVolts = m_driveTalon.getMotorVoltage();
    m_driveCurrent = m_driveTalon.getStatorCurrent();

    // setup turn values
    m_turnTalon.setPosition(0.0);

    m_turnPosition = m_turnTalon.getPosition();
    m_turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_turnTalon, m_turnTalon.getPosition());
    m_turnVelocity = m_turnTalon.getVelocity();
    m_turnAppliedVolts = m_turnTalon.getMotorVoltage();
    m_turnCurrent = m_turnTalon.getStatorCurrent();

    // setup refresh rates on all inputs
    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, m_drivePosition, m_turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_driveVelocity,
        m_driveAppliedVolts,
        m_driveCurrent,
        m_turnAbsolutePosition,
        m_turnVelocity,
        m_turnAppliedVolts,
        m_turnCurrent);
    m_driveTalon.optimizeBusUtilization();
    m_turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
            m_drivePosition,
            m_driveVelocity,
            m_driveAppliedVolts,
            m_driveCurrent,
            m_turnAbsolutePosition,
            m_turnPosition,
            m_turnVelocity,
            m_turnAppliedVolts,
            m_turnCurrent);

    inputs.setDrivePositionRad(
            Units.rotationsToRadians(m_drivePosition.getValueAsDouble()) / m_moduleConstants.DRIVE_GEAR_RATIO());
    inputs.setDriveVelocityRadPerSec(
            Units.rotationsToRadians(m_driveVelocity.getValueAsDouble()) / m_moduleConstants.DRIVE_GEAR_RATIO());
    inputs.setDriveAppliedVolts(m_driveAppliedVolts.getValueAsDouble());
    inputs.setDriveCurrentAmps(new double[] {m_driveCurrent.getValueAsDouble()});

    inputs.setTurnAbsolutePosition(Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble())
        .minus(m_moduleConstants.ENCODER_OFFSET()));
    inputs.setTurnPosition(Rotation2d.fromRotations((
            m_turnPosition.getValueAsDouble() / m_moduleConstants.TURNING_GEAR_RATIO())));
    inputs.setTurnVelocityRadPerSec(
            Units.rotationsToRadians(m_turnVelocity.getValueAsDouble()) / m_moduleConstants.TURNING_GEAR_RATIO());
    inputs.setTurnAppliedVolts(m_turnAppliedVolts.getValueAsDouble());
    inputs.setTurnCurrentAmps(new double[] {m_turnCurrent.getValueAsDouble()});

    inputs.setOdometryDrivePositionsRad(m_drivePositionQueue.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / m_moduleConstants.DRIVE_GEAR_RATIO())
        .toArray());
    inputs.setOdometryTurnPositions(m_turnPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromRotations(value / m_moduleConstants.DRIVE_GEAR_RATIO()))
        .toArray(Rotation2d[]::new));
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnTalon.setControl(new VoltageOut(volts));
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
