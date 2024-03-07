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

import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getKrakenX60(1), ModuleConstants.GearRatios.L3.ratio, 0.025);
  private DCMotorSim turnSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), ModuleConstants.GearRatios.TURN.ratio, 0.004);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final PIDController m_driveController =
      new PIDController(Constants.DriveConstants.DRIVE_FB_GAINS[0], 0.0, 0.0);
  private final PIDController m_turnController =
      new PIDController(Constants.DriveConstants.TURN_FB_GAINS[0], 0.0, 0.0);

  private final ModuleConstants m_moduleConstants;

  public ModuleIOSim(ModuleConstants constants) {
    m_moduleConstants = constants;

    m_turnController.enableContinuousInput(0.0, 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};

    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVelocityMPS(double mps) {
    double rps = (mps / m_moduleConstants.WHEEL_CURCUMFERENCE_METERS()) * m_moduleConstants.DRIVE_GEAR_RATIO();
    driveAppliedVolts = m_driveController.calculate(driveSim.getAngularVelocityRPM() / 60.0, rps);
    driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnPositionRots(double rotations) {
    turnAppliedVolts = m_turnController.calculate(
        turnSim.getAngularPositionRotations(), rotations);
    turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public ModuleConstants getModuleConstants() {
    return m_moduleConstants;
  }
}
