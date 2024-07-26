package frc.robot.commands;

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import lib.utils.AimbotUtils;
import lib.utils.FieldConstants;
import lib.utils.FieldRelativeAccel;
import lib.utils.FieldRelativeSpeed;
import org.littletonrobotics.junction.Logger;


public class AimbotCommand extends Command {
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final XboxController m_driverController;

  private final boolean m_runKicker;
  private final boolean m_pass;

  public AimbotCommand(ArmSubsystem armSubsystem,
                       DriveSubsystem driveSubsystem,
                       ShooterSubsystem shooterSubsystem,
                       XboxController xboxController,
                       boolean runKicker) {
    this (armSubsystem, driveSubsystem, shooterSubsystem, xboxController, runKicker, false);
  }

  public AimbotCommand(ArmSubsystem armSubsystem,
                       DriveSubsystem driveSubsystem,
                       ShooterSubsystem shooterSubsystem,
                       XboxController driverController,
                       boolean runKicker,
                       boolean pass) {
    this.m_armSubsystem = armSubsystem;
    this.m_driveSubsystem = driveSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_driverController = driverController;

    m_runKicker = runKicker;
    m_pass = pass;

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.m_armSubsystem, this.m_driveSubsystem, this.m_shooterSubsystem);
  }

  @Override
  public void execute() {
    double x = -DriveCommands.setSensitivity(-m_driverController.getLeftY(), 0.25);
    double y = -DriveCommands.setSensitivity(-m_driverController.getLeftX(), 0.25);

    x = MathUtil.applyDeadband(x, 0.1);
    y = MathUtil.applyDeadband(y, 0.1);

    double o = DriveCommands.setSensitivity(-m_driverController.getRightX(), 0.15) * 0.75;
    o = MathUtil.applyDeadband(o, 0.1);

    Rotation2d heading;

    // if red change heading goal
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
      heading = m_driveSubsystem.getRotation();
    } else {
      heading = m_driveSubsystem.getRotation().plus(Rotation2d.fromDegrees(180));
    }

    if (m_driveSubsystem.useAutoControl()) {
      Translation3d target = FieldConstants.CENTER_SPEAKER;

      Logger.recordOutput("Aimbot/Target", target);

      // get our desired rotation and error from it
      Rotation2d desiredRotation =
          AimbotUtils.getDrivebaseAimingAngle(m_driveSubsystem.getVisionPose(), target);
      x = MathUtil.clamp(x, -0.45, 0.45);
      y = MathUtil.clamp(y, -0.45, 0.45);

      // if we're far from our setpoint, move faster
      double omega = m_driveSubsystem.alignToAngle(desiredRotation);

      // Convert to field relative speeds & send command
      m_driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
          x * Constants.DriveConstants.MAX_LINEAR_SPEED,
          y * Constants.DriveConstants.MAX_LINEAR_SPEED,
          omega,// - (omegaFF * 0.25),
          heading
      ));

      m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM);
      m_shooterSubsystem.runShooterVelocity(m_runKicker).execute();

      Logger.recordOutput("Aimbot/At Speed", m_shooterSubsystem.atSpeed());
      Logger.recordOutput("Aimbot/At Rotation", m_driveSubsystem.getThetaError() < 20.0);
      Logger.recordOutput("Aimbot/Has Note", m_shooterSubsystem.hasPiece());

      // set shooter speeds and rumble controller
      if (m_shooterSubsystem.atSpeed() && m_driveSubsystem.getThetaError() < 20.0) {
        m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
      } else {
        m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      }
    } else {
      m_driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
          x * m_driveSubsystem.getMaxLinearSpeedMetersPerSec(),
          y * m_driveSubsystem.getMaxLinearSpeedMetersPerSec(),
          o * m_driveSubsystem.getMaxAngularSpeedRadPerSec(),
          heading
      ));

      m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.BACKUP_SHOT);
      m_shooterSubsystem.runShooterVelocity(m_runKicker, () -> 4500, () -> 3750).execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopWithX();
    m_shooterSubsystem.setShooterPowerLeft(0.0);
    m_shooterSubsystem.setShooterPowerRight(0.0);
    m_shooterSubsystem.setKickerPower(0.0);
    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
    m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }
}
