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

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_turnSim;

  private final ModuleConstants m_moduleConstants; // use this mainly for module id

  private final PIDController m_turnPid = new PIDController(0.0, 0.0, 0.0);
  private final PIDController m_drivePid = new PIDController(0.0, 0.0, 0.0);

  private final PidProperty m_turnProperty;
  private final PidProperty m_driveProperty;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(ModuleConstants moduleConstants) {
    m_moduleConstants = moduleConstants;
    // create sims for motors
    m_driveSim = new DCMotorSim(DCMotor.getFalcon500(1), m_moduleConstants.DRIVE_GEAR_RATIO(), 0.025);
    m_turnSim = new DCMotorSim(DCMotor.getFalcon500(1), m_moduleConstants.TURNING_GEAR_RATIO(), 0.004);

    // setup default PID values for "onboard" controllers
    m_turnProperty = new WpiPidPropertyBuilder("Sim Turn PID", false, m_turnPid)
            .addP(m_moduleConstants.TURN_KP())
            .addI(m_moduleConstants.TURN_KI())
            .addD(m_moduleConstants.TURN_KD())
            .build();

    m_driveProperty = new WpiPidPropertyBuilder("Sim Drive PID", false, m_drivePid)
            .addP(m_moduleConstants.DRIVE_KP())
            .addI(m_moduleConstants.DRIVE_KI())
            .addD(m_moduleConstants.DRIVE_KD())
            .build();

    // force update the values
    m_turnProperty.updateIfChanged(true);
    m_driveProperty.updateIfChanged(true);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // update the simulation
    m_driveSim.update(LOOP_PERIOD_SECS);
    m_turnSim.update(LOOP_PERIOD_SECS);

    // update pid controllers
    m_turnProperty.updateIfChanged();
    m_driveProperty.updateIfChanged();

    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(m_driveSim.getCurrentDrawAmps())};

    inputs.setTurnAbsolutePosition(new Rotation2d(m_turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition));
    inputs.setTurnPosition(new Rotation2d(m_turnSim.getAngularPositionRad()));
    inputs.setTurnVelocityRadPerSec(m_turnSim.getAngularVelocityRadPerSec());
    inputs.setTurnAppliedVolts(turnAppliedVolts);
    inputs.setTurnCurrentAmps(new double[] {Math.abs(m_turnSim.getCurrentDrawAmps())});

    inputs.setOdometryDrivePositionsRad(new double[] {inputs.getDrivePositionRad()});
    inputs.setOdometryTurnPositions(new Rotation2d[] {inputs.getTurnPosition()});
  }

  @Override
  public void setDriveVelocityMPS(double mps) {
    double rps = (mps / m_moduleConstants.WHEEL_CURCUMFERENCE_METERS()) * m_moduleConstants.DRIVE_GEAR_RATIO();
    double volts = m_drivePid.calculate(m_driveSim.getAngularVelocityRPM() / 60, rps);
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnPositionDegs(double degrees) {
    double rots = degrees / 360;
    double volts = m_turnPid.calculate(m_turnSim.getAngularPositionRotations(), rots);
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public ModuleConstants getModuleConstants() {
    return  m_moduleConstants;
  }
}
