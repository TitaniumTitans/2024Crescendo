package frc.robot.commands;

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import lib.utils.AimbotUtils;


public class AimbotCommand extends Command {
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final XboxController m_driverController;

  private final PIDController m_smallController;
  private final PIDController m_fastController;

  private final PidProperty m_smallProperty;
  private final PidProperty m_fastProperty;

  private boolean m_runKicker;

  public AimbotCommand(ArmSubsystem armSubsystem,
                       DriveSubsystem driveSubsystem,
                       ShooterSubsystem shooterSubsystem,
                       XboxController driverController,
                       boolean runKicker) {
    this.m_armSubsystem = armSubsystem;
    this.m_driveSubsystem = driveSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_driverController = driverController;

    m_smallController = new PIDController(0.0, 0.0, 0.0);
    m_fastController = new PIDController(0.0, 0.0, 0.0);

    m_smallProperty = new WpiPidPropertyBuilder("Drive/Aimbot Small", false, m_smallController)
            .addP(1.0)
            .addI(0.0)
            .addD(0.0)
            .build();
    m_fastProperty = new WpiPidPropertyBuilder("Drive/Aimbot Fast", false, m_fastController)
            .addP(3.0)
            .addI(0.0)
            .addD(0.0)
            .build();

    m_runKicker = runKicker;

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.m_armSubsystem, this.m_driveSubsystem, this.m_shooterSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_smallProperty.updateIfChanged();
    m_fastProperty.updateIfChanged();

    // get our desired rotation and error from it
    double desiredRotationDegs = AimbotUtils.getDrivebaseAimingAngle(m_driveSubsystem.getVisionPose()).getDegrees();
    double error = Math.abs(desiredRotationDegs - m_driveSubsystem.getRotation().getDegrees());

    // if we're far from our setpoint, move faster
    double omega;
    if (error > 15.0) {
      omega = m_fastController.calculate(m_driveSubsystem.getRotation().getDegrees(), desiredRotationDegs);
    } else {
      omega = m_smallController.calculate(m_driveSubsystem.getRotation().getDegrees(), desiredRotationDegs);
    }

    double x = -DriveCommands.setSensitivity(-m_driverController.getLeftY(), 0.25);
    double y = -DriveCommands.setSensitivity(-m_driverController.getLeftX(), 0.25);


    Rotation2d heading;

    // if red change heading goal
    if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      heading = m_driveSubsystem.getRotation().plus(Rotation2d.fromDegrees(180));
    } else {
      heading = m_driveSubsystem.getRotation();
    }

    // Convert to field relative speeds & send command
    m_driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            x * Constants.DriveConstants.MAX_LINEAR_SPEED,
            y * Constants.DriveConstants.MAX_LINEAR_SPEED,
            omega * Constants.DriveConstants.MAX_ANGULAR_SPEED,
            heading
    ));

    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM);
    m_shooterSubsystem.runShooterVelocity(m_runKicker);

    // set shooter speeds and rumble controller
    if (m_shooterSubsystem.atSpeed() && error < 10.0) {
      m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    } else {
      m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopWithX();
    m_shooterSubsystem.runShooterVelocity(false, 0.0, 0.0);
    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
    m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }
}
