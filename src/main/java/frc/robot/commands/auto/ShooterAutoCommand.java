package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import lib.utils.AimbotUtils;
import lib.utils.FieldConstants;


public class ShooterAutoCommand extends Command {
  private final ArmSubsystem m_armSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  private final Timer m_timer;

  private boolean m_previousHadPiece;
  private boolean m_hasChanged;

  public ShooterAutoCommand(ArmSubsystem armSubsystem,
                            ShooterSubsystem shooterSubsystem,
                            DriveSubsystem driveSubsystem) {
    this.m_armSubsystem = armSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_driveSubsystem = driveSubsystem;

    m_timer = new Timer();
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.m_armSubsystem, this.m_shooterSubsystem, driveSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_previousHadPiece = m_shooterSubsystem.hasPiece();
  }

  @Override
  public void execute() {
    // Calculate output to align to speaker
    double omega = m_driveSubsystem.alignToAngle(AimbotUtils.getDrivebaseAimingAngle(m_driveSubsystem.getVisionPose()));
    m_driveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

    // only actually shoot if we're aligned close enough to speaker and flywheels are at speed
    m_shooterSubsystem.runShooterVelocity(m_shooterSubsystem.atSpeed()
        && omega < 1).execute();
    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM);

    // check to see if the state of having a note has changed, mark if it has
    if (m_previousHadPiece != m_shooterSubsystem.hasPiece()) {
      m_hasChanged = true;
    }

    m_previousHadPiece = m_shooterSubsystem.hasPiece();
  }

  @Override
  public boolean isFinished() {
    // if magazine is empty, we've detected a note has run through the system, and we're not taking too
    // short or too long to run

    // too long check may change as system gets tuned
    return (!m_shooterSubsystem.hasPiece() && m_hasChanged
        && m_timer.hasElapsed(0.5)) || m_timer.hasElapsed(5.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooterPowerFactory(0.0, 0.0, 0.0).execute();
    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
  }
}
