package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import lib.utils.AllianceFlipUtil;

public class DavidDriveCommand extends Command {

  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;
  private double m_lastAngle;

  public DavidDriveCommand(DriveSubsystem driveSubsystem, CommandXboxController joystick) {
    m_drive = driveSubsystem;
    m_controller = joystick;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_lastAngle = m_drive.getVisionPose().getRotation()
        .plus(Rotation2d.fromDegrees(180)).getDegrees();
  }

  @Override
  public void execute() {
    double xRight = MathUtil.applyDeadband(-m_controller.getRightX(), 0.1);
    double yRight = MathUtil.applyDeadband(-m_controller.getRightY(), 0.1);
    double xLeft = MathUtil.applyDeadband(-m_controller.getLeftX(), 0.1);
    double yLeft = MathUtil.applyDeadband(-m_controller.getLeftY(), 0.1);

    double joyStickAngle;
    if (Math.sqrt(xRight * xRight + yRight * yRight) > 0.75) {
      joyStickAngle = Math.toDegrees(Math.atan2(xRight, yRight));
      m_lastAngle = joyStickAngle;
    } else {
      joyStickAngle = m_lastAngle;
    }

    joyStickAngle += DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue
            ? 0 : 180;

    if (Constants.DriveConstants.USE_DAVID_DRIVE.getValue()) {
      m_drive.davidDrive(
              yLeft * m_drive.getMaxLinearSpeedMetersPerSec(),
              xLeft * m_drive.getMaxLinearSpeedMetersPerSec(),
              joyStickAngle + 180);
    } else {
      DriveCommands.joystickDrive(
              m_drive,
              () -> yLeft,
              () -> xLeft,
              () -> xRight
      ).execute();
    }
  }
}
