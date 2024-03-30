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

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.auto.IntakeControlCommand;
import frc.robot.commands.auto.ShooterAutoCommand;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import lib.utils.AllianceFlipUtil;
import lib.utils.FieldConstants;
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
  public final ArmSubsystem m_armSubsystem;
  private final ClimberSubsystem m_climber;
  private final LedSubsystem m_led;

  // Controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  private final LoggedDashboardNumber m_armIncrement = new LoggedDashboardNumber("Arm/Increment value", 1);
  private final LoggedDashboardNumber m_leftPower = new LoggedDashboardNumber("Shooter/Left Power", 2250);
  private final LoggedDashboardNumber m_rightPower = new LoggedDashboardNumber("Shooter/Right Power", 2250);

  // Dashboard inputs
  private final AutoFactory m_autonFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        m_driveSubsystem = new DriveSubsystem(
            new GyroIOPigeon2(true),
            new ModuleIOTalonFX(DriveConstants.FL_MOD_CONSTANTS),
            new ModuleIOTalonFX(DriveConstants.FR_MOD_CONSTANTS),
            new ModuleIOTalonFX(DriveConstants.BL_MOD_CONSTANTS),
            new ModuleIOTalonFX(DriveConstants.BR_MOD_CONSTANTS),
            new VisionSubsystem[]{
                new VisionSubsystem("RightCamera", DriveConstants.RIGHT_CAMERA_TRANSFORMATION),
                new VisionSubsystem("LeftCamera", DriveConstants.LEFT_CAMERA_TRANSFORMATION),
                new VisionSubsystem("IntakeCamera", DriveConstants.INTAKE_CAMERA_TRANSFORMATION)
            }
            );
        m_shooter = new ShooterSubsystem(new ShooterIOKraken());
        m_climber = new ClimberSubsystem(new ClimberIOKraken() {});
        m_armSubsystem = new ArmSubsystem(new ArmIOKraken(),
            m_driveSubsystem::getVisionPose, m_climber::getClimberLock, m_climber::getClimberHeight);
      }
      case SIM -> {
      // Sim robot, instantiate physics sim IO implementations
        m_driveSubsystem =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FL_MOD_CONSTANTS),
                new ModuleIOSim(DriveConstants.FR_MOD_CONSTANTS),
                new ModuleIOSim(DriveConstants.BL_MOD_CONSTANTS),
                new ModuleIOSim(DriveConstants.BR_MOD_CONSTANTS));
        m_shooter = new ShooterSubsystem(new ShooterIOSim());
        m_climber = new ClimberSubsystem(new ClimberIO() {});
        m_armSubsystem = new ArmSubsystem(new ArmIOSim(),
            m_driveSubsystem::getVisionPose, m_climber::getClimberLock, m_climber::getClimberHeight);
      }
      default -> {
        // Replayed robot, disable IO implementations
        m_driveSubsystem =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_shooter = new ShooterSubsystem(new ShooterIO() {});
        m_armSubsystem = new ArmSubsystem(new ArmIO() {});
        m_climber = new ClimberSubsystem(new ClimberIO() {});
      }
    }
    m_led = new LedSubsystem(m_shooter::hasPiece);

    // configure named commands for auto
    configureNamedCommands();
    configureDashboard();

    m_autonFactory = new AutoFactory();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * Joystick} or {@link XboxController}), and then passing it to a {@link
   * JoystickButton}.
   */
  private void configureButtonBindings() {
    /** trigger setup */
    Trigger intakeTrigger = m_driverController.y().and(m_driverController.x().negate())
        .and(m_driverController.a().negate()) // make sure we don't amp
        .and(m_driverController.b().negate())
        .and(m_driverController.leftTrigger().negate());

    Trigger spinUpTrigger = m_driverController.x().and(m_driverController.y().negate())
        .and(m_driverController.a().negate()) // make sure we don't amp
        .and(m_driverController.b().negate());

    Trigger passSpinUpTrigger = m_driverController.leftTrigger()
        .and(spinUpTrigger.negate())
        .and(m_driverController.y().negate());

    Trigger passTrigger = m_driverController.leftTrigger()
        .and(spinUpTrigger.negate())
        .and(m_driverController.y());

    Trigger shootTrigger = m_driverController.x().and(m_driverController.y())
        .and(m_driverController.a().negate()) // make sure we don't amp
        .and(m_driverController.b().negate())
        .and(m_driverController.leftTrigger().negate());

    Trigger ampLineupTrigger = m_driverController.b().and(m_driverController.a().negate())
        .debounce(0.1, Debouncer.DebounceType.kBoth);

    Trigger ampDepositeTrigger = m_driverController.b().and(m_driverController.a())
        .and(spinUpTrigger.negate()) // make sure we don't amp while trying to do anything else
        .and(shootTrigger.negate())
        .and(intakeTrigger.negate())
        .debounce(0.1, Debouncer.DebounceType.kBoth);

    /** driver controller */

    intakeTrigger.whileTrue(m_shooter.intakeCommand(0.75, 0.5, 0.1)
        .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.INTAKE)));

    spinUpTrigger.whileTrue(
        new AimbotCommand(m_armSubsystem, m_driveSubsystem, m_shooter, m_driverController.getHID(), false));
    shootTrigger.whileTrue(
        new AimbotCommand(m_armSubsystem, m_driveSubsystem, m_shooter, m_driverController.getHID(), true));

    ampLineupTrigger.whileTrue(m_driveSubsystem.pathfollowFactory(FieldConstants.AMP_LINEUP)
        .finallyDo(() -> m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AMP).schedule()))
        .whileFalse(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.STOW));

    ampDepositeTrigger.whileTrue(Commands.runEnd(() -> m_shooter.setKickerPower(-0.75),
        () -> m_shooter.setKickerPower(0.0),
        m_shooter)
        .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AMP)));

    passSpinUpTrigger.whileTrue(
        m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.PASS)
            .alongWith(m_shooter.runShooterVelocity(false, () -> 3500, () -> 3500)));

    passTrigger.whileTrue(
        m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.PASS)
            .alongWith(m_shooter.runShooterVelocity(true, () -> 3500, () -> 3500)));

    m_driverController.pov(180).whileTrue(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AMP));
    m_driverController.pov(0).whileTrue(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.ANTI_DEFENSE));

    m_driverController.pov(90).whileTrue(
            Commands.runEnd(() -> {
                      m_shooter.setIntakePower(-0.75);
                      m_shooter.setKickerPower(-0.75);
                      m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.INTAKE);
                    },
                    () -> {
                      m_shooter.setIntakePower(0.0);
                      m_shooter.setKickerPower(0.0);
                      m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
                    },
                    m_shooter));

    // 96.240234375
    // 60.029296875
    // 2250

    m_driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()
        ));
    m_driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                            new Pose2d(m_driveSubsystem.getVisionPose().getTranslation(),
                            Rotation2d.fromDegrees(180.0)))),
                    m_driveSubsystem)
                .ignoringDisable(true));


    /** Operator controller */
    m_operatorController.leftTrigger().whileTrue(
        m_shooter.runShooterVelocity(false, () -> 1400, () -> 1400));
    m_operatorController.rightTrigger().whileTrue(
        m_shooter.runShooterVelocity(true, () -> 1400, () -> 1400));

    m_operatorController.leftBumper().whileTrue(m_climber.setClimberPosition(-360.0 * 3.0));
    m_operatorController.rightBumper().whileTrue(m_climber.setClimberPosition(1230.0)
        .unless(() -> m_armSubsystem.getArmState() == ArmSubsystem.ArmState.SCORE_TRAP));
    m_operatorController.y().whileTrue(m_climber.setClimberPosition(30.0)
        .unless(() -> m_armSubsystem.getArmState() == ArmSubsystem.ArmState.SCORE_TRAP));

    m_operatorController.pov(0).whileTrue(m_climber.setClimberPowerFactory(0.5));
    m_operatorController.pov(180).whileTrue(m_climber.setClimberPowerFactory(-0.5));
//    m_operatorController.pov(90).whileTrue(m_climber.setRightClimberPowerFactory(0.5));
//    m_operatorController.pov(270).whileTrue(m_climber.setRightClimberPowerFactory(-0.5));

    m_operatorController.a()
        .onTrue(Commands.runOnce(() -> m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.PREPARE_TRAP),
            m_armSubsystem));
    m_operatorController.b()
        .onTrue(Commands.runOnce(() -> m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.SCORE_TRAP),
            m_armSubsystem));
    m_operatorController.x()
        .onTrue(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.STOW));

    // arm 45.0
    // wrist 123.5

    m_operatorController.start().onTrue(m_climber.resetClimber());
  }

  /**
   * Use this method to configure any named commands needed for PathPlanner autos
   */
  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake", new IntakeControlCommand(m_armSubsystem, m_shooter));

    NamedCommands.registerCommand("AimAndShoot", new ShooterAutoCommand(m_armSubsystem, m_shooter, m_driveSubsystem));
  }

  private void configureDashboard() {
    ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");

    commandTab.add("Disable Arm Brake", m_armSubsystem.enableBrakeMode(false));
    commandTab.add("Enable Arm Brake", m_armSubsystem.enableBrakeMode(true));
    commandTab.add("Reset Climber Lock", m_climber.resetClimberLock());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonFactory.getSelectedAutonomous();
  }

  public void resetClimberLock() {
    m_climber.resetClimberLock().schedule();
  }
}
