package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;


public class AimBotCommand extends Command {
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  public AimBotCommand(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem) {
    this.m_armSubsystem = armSubsystem;
    this.m_driveSubsystem = driveSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.m_armSubsystem, this.m_driveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopWithX();
  }
}
