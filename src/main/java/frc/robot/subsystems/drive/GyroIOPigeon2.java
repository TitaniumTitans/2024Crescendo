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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.PhoenixOdometryThread;
import frc.robot.subsystems.drive.module.SparkMaxOdometryThread;

import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(13, "canivore");
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2(boolean phoenixDrive) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    if (phoenixDrive) {
      yawPositionQueue =
          PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
    } else {
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(() -> pigeon.getYaw().getValueAsDouble());
    }
  }

  @Override
  public void updateInputs(GyroIOInputsAutoLogged inputs) {
    inputs.setConnected(BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK));
    inputs.setYawPosition(Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble()));
    inputs.setYawVelocityRadPerSec(Units.degreesToRadians(yawVelocity.getValueAsDouble()));

    inputs.setOdometryYawPositions(yawPositionQueue.stream()
        .map(Rotation2d::fromDegrees)
        .toArray(Rotation2d[]::new));
    yawPositionQueue.clear();
  }

  @Override
  public void resetGyro(double degrees) {
    pigeon.setYaw(degrees / 360);
  }
}
