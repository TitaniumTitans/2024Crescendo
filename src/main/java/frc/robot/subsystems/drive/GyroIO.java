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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  class GyroIOInputs {
    protected boolean connected = false;
    protected Rotation2d yawPosition = new Rotation2d();
    protected Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    protected double yawVelocityRadPerSec = 0.0;

    public boolean isConnected() {
      return connected;
    }

    public void setConnected(boolean connected) {
      this.connected = connected;
    }

    public Rotation2d getYawPosition() {
      return yawPosition;
    }

    public void setYawPosition(Rotation2d yawPosition) {
      this.yawPosition = yawPosition;
    }

    public Rotation2d[] getOdometryYawPositions() {
      return odometryYawPositions;
    }

    public void setOdometryYawPositions(Rotation2d[] odometryYawPositions) {
      this.odometryYawPositions = odometryYawPositions;
    }

    public double getYawVelocityRadPerSec() {
      return yawVelocityRadPerSec;
    }

    public void setYawVelocityRadPerSec(double yawVelocityRadPerSec) {
      this.yawVelocityRadPerSec = yawVelocityRadPerSec;
    }
  }

  default void updateInputs(GyroIOInputs inputs) {}

  default void resetGyro(double degrees) {}
}
