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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    protected double drivePositionRad = 0.0;
    protected double driveVelocityRadPerSec = 0.0;
    protected double driveAppliedVolts = 0.0;
    protected double[] driveCurrentAmps = new double[] {};

    protected Rotation2d turnAbsolutePosition = new Rotation2d();
    protected Rotation2d turnPosition = new Rotation2d();
    protected double turnVelocityRadPerSec = 0.0;
    protected double turnAppliedVolts = 0.0;
    protected double[] turnCurrentAmps = new double[] {};

    protected double[] odometryDrivePositionsRad = new double[] {};
    protected Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    public double getDrivePositionRad() {
      return drivePositionRad;
    }

    public void setDrivePositionRad(double drivePositionRad) {
      this.drivePositionRad = drivePositionRad;
    }

    public double getDriveVelocityRadPerSec() {
      return driveVelocityRadPerSec;
    }

    public void setDriveVelocityRadPerSec(double driveVelocityRadPerSec) {
      this.driveVelocityRadPerSec = driveVelocityRadPerSec;
    }

    public double getDriveAppliedVolts() {
      return driveAppliedVolts;
    }

    public void setDriveAppliedVolts(double driveAppliedVolts) {
      this.driveAppliedVolts = driveAppliedVolts;
    }

    public double[] getDriveCurrentAmps() {
      return driveCurrentAmps;
    }

    public void setDriveCurrentAmps(double[] driveCurrentAmps) {
      this.driveCurrentAmps = driveCurrentAmps;
    }

    public Rotation2d getTurnAbsolutePosition() {
      return turnAbsolutePosition;
    }

    public void setTurnAbsolutePosition(Rotation2d turnAbsolutePosition) {
      this.turnAbsolutePosition = turnAbsolutePosition;
    }

    public Rotation2d getTurnPosition() {
      return turnPosition;
    }

    public void setTurnPosition(Rotation2d turnPosition) {
      this.turnPosition = turnPosition;
    }

    public double getTurnVelocityRadPerSec() {
      return turnVelocityRadPerSec;
    }

    public void setTurnVelocityRadPerSec(double turnVelocityRadPerSec) {
      this.turnVelocityRadPerSec = turnVelocityRadPerSec;
    }

    public double getTurnAppliedVolts() {
      return turnAppliedVolts;
    }

    public void setTurnAppliedVolts(double turnAppliedVolts) {
      this.turnAppliedVolts = turnAppliedVolts;
    }

    public double[] getTurnCurrentAmps() {
      return turnCurrentAmps;
    }

    public void setTurnCurrentAmps(double[] turnCurrentAmps) {
      this.turnCurrentAmps = turnCurrentAmps;
    }

    public double[] getOdometryDrivePositionsRad() {
      return odometryDrivePositionsRad;
    }

    public void setOdometryDrivePositionsRad(double[] odometryDrivePositionsRad) {
      this.odometryDrivePositionsRad = odometryDrivePositionsRad;
    }

    public Rotation2d[] getOdometryTurnPositions() {
      return odometryTurnPositions;
    }

    public void setOdometryTurnPositions(Rotation2d[] odometryTurnPositions) {
      this.odometryTurnPositions = odometryTurnPositions;
    }
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  default void setDriveVelocityMPS(double mps) {}

  /** Run the turn motor at the specified voltage. */
  default void setTurnPositionDegs(double degrees) {}

  /** Enable or disable brake mode on the drive motor. */
  default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  default void setTurnBrakeMode(boolean enable) {}

  // Used to pass moduleConstants
  default ModuleConstants getModuleConstants() {
    throw new UnsupportedOperationException("getModuleConstants() not implemented");
  }
}
