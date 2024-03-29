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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import lib.utils.AimbotUtils;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
//    AtomicReference<Rotation2d> prevheading = new AtomicReference<>(driveSubsystem.getRotation());
    return Commands.run(
        () -> {
          double xInput = setSensitivity(xSupplier.getAsDouble(), 0.25);
          double yInput = setSensitivity(ySupplier.getAsDouble(), 0.25);
          double omegaInput = setSensitivity(omegaSupplier.getAsDouble(), 0.0) * 0.5;

          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xInput, yInput), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xInput, yInput);
          double omega = MathUtil.applyDeadband(omegaInput, DEADBAND);

//          if (omegaInput < DEADBAND) {
//            omega = driveSubsystem.alignToAngle(prevheading.get());
//            prevheading.set(driveSubsystem.getRotation());
//          }

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          Rotation2d heading = new Rotation2d();

          // if red change heading goal
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            heading = driveSubsystem.getRotation().plus(Rotation2d.fromDegrees(180));
          } else {
            heading = driveSubsystem.getRotation();
          }

          // Convert to field relative speeds & send command
          driveSubsystem.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                  omega * driveSubsystem.getMaxAngularSpeedRadPerSec()
                          * Constants.DriveConstants.TURNING_SPEED.getValue(),
                  heading));
        },
            driveSubsystem);
  }

  public static Command alignmentDrive(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation3d> point) {
    return Commands.run(() -> {
      double xInput = setSensitivity(xSupplier.getAsDouble(), 0.25);
      double yInput = setSensitivity(ySupplier.getAsDouble(), 0.25);

      // Apply deadband
      double linearMagnitude =
          MathUtil.applyDeadband(
              Math.hypot(xInput, yInput), DEADBAND);
      Rotation2d linearDirection =
          new Rotation2d(xInput, yInput);

      // Calculate omega
      double omega =
          driveSubsystem.alignToAngle(AimbotUtils.getDrivebaseAimingAngle(driveSubsystem.getVisionPose()));

      // Calcaulate new linear velocity
      Translation2d linearVelocity =
          new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
              .getTranslation();

      Rotation2d heading = new Rotation2d();

      // if red change heading goal
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        heading = driveSubsystem.getRotation().plus(Rotation2d.fromDegrees(180));
      } else {
        heading = driveSubsystem.getRotation();
      }

      // Convert to field relative speeds & send command
      driveSubsystem.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
              omega * driveSubsystem.getMaxAngularSpeedRadPerSec(),
              heading));

    }, driveSubsystem);
  }

  public static double setSensitivity(double x, double sensitivity) {
    return sensitivity * x + ((1.0 - sensitivity) * Math.pow(x, 3.0));
  }
}
