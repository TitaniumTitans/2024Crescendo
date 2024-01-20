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

import edu.wpi.first.math.MathUtil;
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

  private final DCMotorSim m_driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private final DCMotorSim m_turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

  private final ModuleConstants m_moduleConstants; // use this mainly for module id


  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(ModuleConstants moduleConstants) {
    m_moduleConstants = moduleConstants;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    m_driveSim.update(LOOP_PERIOD_SECS);
    m_turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(m_driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(m_turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(m_turnSim.getCurrentDrawAmps())};

    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public ModuleConstants getModuleConstants() {
    return  m_moduleConstants;
  }
}
