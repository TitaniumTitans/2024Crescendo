package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class IntakeControlCommand extends Command {
  private final ArmSubsystem m_armSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final Timer m_timer;
  private boolean hasPieceRising;

  public IntakeControlCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
    m_armSubsystem = armSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_timer = new Timer();
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_armSubsystem, m_shooterSubsystem);
  }

  @Override
  public void initialize() {
    hasPieceRising = false;
    m_timer.reset();
  }

  @Override
  public void execute() {
    if(!m_shooterSubsystem.hasPiece() && !hasPieceRising) {
      // run intake and kicker wheels in
      m_shooterSubsystem.setIntakePower(0.95);
      m_shooterSubsystem.setKickerPower(0.35);
      m_shooterSubsystem.setShooterPowerLeft(-0.1);
      m_shooterSubsystem.setShooterPowerRight(-0.1);
      m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.INTAKE);
    } else if (m_shooterSubsystem.hasPiece() && !m_timer.hasElapsed(0.0001)) {
      // stop running intake
      m_shooterSubsystem.setIntakePower(0.0);
      m_shooterSubsystem.setKickerPower(0.0);
      m_shooterSubsystem.setShooterPowerLeft(0.0);
      m_shooterSubsystem.setShooterPowerRight(0.0);

      // piece detected, mark as we have a piece and start moving up
      m_timer.start();
      m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
    } else if (!m_timer.hasElapsed(0.075) && m_armSubsystem.bothAtSetpoint()) {
      // arm is up, haven't run kickers back yet
      m_shooterSubsystem.setKickerPower(-0.2);
    } else {
      m_shooterSubsystem.setIntakePower(0.0);
      m_shooterSubsystem.setKickerPower(0.0);
      m_shooterSubsystem.setShooterPowerLeft(0.0);
      m_shooterSubsystem.setShooterPowerRight(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    // We won't return true because it's being ran by a .whileTrue() method
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setIntakePower(0.0);
    m_shooterSubsystem.setKickerPower(0.0);
    m_shooterSubsystem.setShooterPowerLeft(0.0);
    m_shooterSubsystem.setShooterPowerRight(0.0);
    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
  }
}
