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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOPrototype;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon1;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOPrototype;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooter;
  private final ClimberSubsystem m_climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      switch (Constants.currentMode) {
          case REAL -> {
              // Real robot, instantiate hardware IO implementations
              m_driveSubsystem = new DriveSubsystem(
                new GyroIOPigeon1(12, true),
                  new ModuleIOTalonFX(Constants.DriveConstants.FL_MOD_CONSTANTS),
                  new ModuleIOTalonFX(Constants.DriveConstants.FR_MOD_CONSTANTS),
                  new ModuleIOTalonFX(Constants.DriveConstants.BL_MOD_CONSTANTS),
                  new ModuleIOTalonFX(Constants.DriveConstants.BR_MOD_CONSTANTS));
              m_shooter = new ShooterSubsystem(new ShooterIOPrototype());
              m_climber = new ClimberSubsystem(new ClimberIOPrototype());
          }
          case SIM -> {
              // Sim robot, instantiate physics sim IO implementations
              m_driveSubsystem =
                new DriveSubsystem(
                  new GyroIO() {
                  },
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
              m_shooter = new ShooterSubsystem(new ShooterIOPrototype());
              m_climber = new ClimberSubsystem(new ClimberIO() {});
          }
          default -> {
              // Replayed robot, disable IO implementations
              m_driveSubsystem =
                      new DriveSubsystem(
                              new GyroIO() {
                              },
                              new ModuleIO() {
                              },
                              new ModuleIO() {
                              },
                              new ModuleIO() {
                              },
                              new ModuleIO() {
                              });
              m_shooter = new ShooterSubsystem(new ShooterIO() {});
              m_climber = new ClimberSubsystem(new ClimberIO() {});
          }
      }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
                m_driveSubsystem, m_driveSubsystem::runCharacterizationVolts, m_driveSubsystem::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
                m_driveSubsystem,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(m_driveSubsystem::stopWithX, m_driveSubsystem));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_driveSubsystem.setPose(
                            new Pose2d(m_driveSubsystem.getPose().getTranslation(), new Rotation2d())),
                            m_driveSubsystem)
                .ignoringDisable(true));

    controller.a().whileTrue(m_shooter.setShooterPower(0.6, 0.6))
            .whileFalse(m_shooter.setShooterPower(0.0, 0.0));

    controller.leftBumper().whileTrue(m_climber.setClimberPowerFactory(-0.25))
            .whileFalse(m_climber.setClimberPowerFactory(0.0));
      controller.rightBumper().whileTrue(m_climber.setClimberPowerFactory(0.25))
              .whileFalse(m_climber.setClimberPowerFactory(0.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
