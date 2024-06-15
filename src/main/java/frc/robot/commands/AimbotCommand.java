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

  private final PIDController m_smallController;
  private final PIDController m_fastController;

  private final PidProperty m_smallProperty;
  private final PidProperty m_fastProperty;

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

    m_smallController = new PIDController(0.0, 0.0, 0.0);
    m_fastController = new PIDController(0.0, 0.0, 0.0);

    m_smallController.enableContinuousInput(-180, 180);
    m_fastController.enableContinuousInput(-180, 180);

    m_smallController.setTolerance(5.0);
    m_fastController.setTolerance(10.0);

    m_smallProperty = new WpiPidPropertyBuilder("Drive/Aimbot Small", true, m_smallController)
            .addP(0.03)
            .addI(0.001)
            .addD(0.004)
            .build();
    m_fastProperty = new WpiPidPropertyBuilder("Drive/Aimbot Fast", true, m_fastController)
            .addP(0.1)
            .addI(0.0)
            .addD(0.002)
            .build();

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
      m_smallProperty.updateIfChanged();
      m_fastProperty.updateIfChanged();

      // get the robots velocity and acceleration
      FieldRelativeSpeed fieldRelativeSpeed = m_driveSubsystem.getFieldRelativeVelocity();
      FieldRelativeAccel fieldRelativeAccel = m_driveSubsystem.getFieldRelativeAcceleration();

      // TODO make an actual equation for shot time based on distance
      double shotTime = 0.5;

      Translation3d target = FieldConstants.CENTER_SPEAKER;
      Translation3d movingTarget = new Translation3d();

      // loop over movement calcs to better adjust for acceleration
      if (true) {
        for (int i = 0; i < 1; i++) {
          double virtualGoalX = target.getX()
              - shotTime * (
              MathUtil.applyDeadband(fieldRelativeSpeed.vx, 0.25)
                  + MathUtil.applyDeadband(
                  fieldRelativeAccel.ax * ShooterConstants.ACCEL_COMP_FACTOR.getValue(), 0.25));

          double virtualGoalY = target.getY()
              - shotTime * (
              MathUtil.applyDeadband(fieldRelativeSpeed.vy, 0.25)
                  + MathUtil.applyDeadband(
                  fieldRelativeAccel.ay * ShooterConstants.ACCEL_COMP_FACTOR.getValue(), 0.25));

          movingTarget = new Translation3d(virtualGoalX, virtualGoalY, 0.0);
        }
      } else {
        movingTarget = target;
      }

      Logger.recordOutput("Aimbot/Target", target);
      Logger.recordOutput("Aimbot/Moving Target", movingTarget);

      Logger.recordOutput("Aimbot/Field Relative Velocity",
          new ChassisSpeeds(
              fieldRelativeSpeed.vx,
              fieldRelativeSpeed.vy,
              fieldRelativeSpeed.omega
          ));

      // get our desired rotation and error from it
      Rotation2d desiredRotation =
          AimbotUtils.getDrivebaseAimingAngle(m_driveSubsystem.getVisionPose(), movingTarget);
      x = MathUtil.clamp(x, -0.45, 0.45);
      y = MathUtil.clamp(y, -0.45, 0.45);

      // if we're far from our setpoint, move faster
      double omega = m_driveSubsystem.alignToAngle(desiredRotation);
      double error = m_driveSubsystem.getThetaError();
//      if (error > 5.0) {
//        omega = m_fastController.calculate(m_driveSubsystem.getRotation().getDegrees(), desiredRotationDegs);
//      } else {
//        omega = m_smallController.calculate(m_driveSubsystem.getRotation().getDegrees(), desiredRotationDegs);
//      }

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
      Logger.recordOutput("Aimbot/At Rotation", error < 20.0);
      Logger.recordOutput("Aimbot/Has Note", m_shooterSubsystem.hasPiece());

      // set shooter speeds and rumble controller
      if (m_shooterSubsystem.atSpeed() && error < 20.0) {
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
