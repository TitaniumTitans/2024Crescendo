package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class ShooterAutoCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ShooterAutoCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
    this.armSubsystem = armSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.armSubsystem, this.shooterSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    shooterSubsystem.runShooterVelocity(shooterSubsystem.atSpeed()).execute();
    armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM);
  }

  @Override
  public boolean isFinished() {
    return !shooterSubsystem.hasPiece();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterPowerFactory(0.0, 0.0, 0.0).execute();
    armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
  }
}
