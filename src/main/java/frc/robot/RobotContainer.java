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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.auto.ShooterAutoCommand;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final AutoFactory m_autonFactory;
  private final LoggedDashboardNumber m_leftRPM = new LoggedDashboardNumber("Shooter/LeftRPM", 3600);
  private final LoggedDashboardNumber m_rightRPM = new LoggedDashboardNumber("Shooter/RightRPM", 3600);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        m_driveSubsystem = new DriveSubsystem(
            new GyroIOPigeon2(true),
            new ModuleIOTalonFX(Constants.DriveConstants.FL_MOD_CONSTANTS),
            new ModuleIOTalonFX(Constants.DriveConstants.FR_MOD_CONSTANTS),
            new ModuleIOTalonFX(Constants.DriveConstants.BL_MOD_CONSTANTS),
            new ModuleIOTalonFX(Constants.DriveConstants.BR_MOD_CONSTANTS),
            new VisionSubsystem[]{
                new VisionSubsystem("RightCamera", DriveConstants.RIGHT_CAMERA_TRANSFORMATION),
                new VisionSubsystem("LeftCamera", DriveConstants.LEFT_CAMERA_TRANSFORMATION)
            }
            );
        m_shooter = new ShooterSubsystem(new ShooterIOKraken(), m_driveSubsystem::getVisionPose);
        m_armSubsystem = new ArmSubsystem(new ArmIOKraken(), m_driveSubsystem::getVisionPose);
        m_climber = new ClimberSubsystem(new ClimberIOKraken() {});
      }
      case PROTO_ARM -> {
        m_driveSubsystem = new DriveSubsystem(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        m_shooter = new ShooterSubsystem(new ShooterIO() {});
        m_armSubsystem = new ArmSubsystem(new ArmIOPrototype());
        m_climber = new ClimberSubsystem(new ClimberIO() {});
      }
      case PROTO_SHOOTER -> {
        m_driveSubsystem = new DriveSubsystem(
            new GyroIO() {
            },
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        m_shooter = new ShooterSubsystem(new ShooterIOKraken());
        m_climber = new ClimberSubsystem(new ClimberIO() {});
        m_armSubsystem = new ArmSubsystem(new ArmIO() {});
      }
      case SIM -> {
//       Sim robot, instantiate physics sim IO implementations
        m_driveSubsystem =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FL_MOD_CONSTANTS),
                new ModuleIOSim(DriveConstants.FR_MOD_CONSTANTS),
                new ModuleIOSim(DriveConstants.BL_MOD_CONSTANTS),
                new ModuleIOSim(DriveConstants.BR_MOD_CONSTANTS));
        m_shooter = new ShooterSubsystem(new ShooterIOSim());
        m_armSubsystem = new ArmSubsystem(new ArmIOSim());
        m_climber = new ClimberSubsystem(new ClimberIO() {});
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
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger intakeTrigger = controller.y().and(controller.x().negate())
        .and(controller.a().negate()) // make sure we don't amp
        .and(controller.b().negate());

    Trigger spinUpTrigger = controller.x().and(controller.y().negate())
        .and(controller.a().negate()) // make sure we don't amp
        .and(controller.b().negate());

    Trigger shootTrigger = controller.x().and(controller.y())
        .and(controller.a().negate()) // make sure we don't amp
        .and(controller.b().negate());

    Trigger ampLineupTrigger = controller.b().and(controller.a().negate())
        .debounce(0.1, Debouncer.DebounceType.kBoth);

    Trigger ampDepositeTrigger = controller.b().and(controller.a())
        .and(spinUpTrigger.negate()) // make sure we don't amp while trying to do anything else
        .and(shootTrigger.negate())
        .and(intakeTrigger.negate())
        .debounce(0.1, Debouncer.DebounceType.kBoth);

    intakeTrigger.whileTrue(m_shooter.intakeCommand(0.75, 0.5, 0.1)
        .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.INTAKE)));

//    spinUpTrigger.whileTrue(m_shooter.runShooterVelocity(false)
//        .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AUTO_AIM))
//        .alongWith(DriveCommands.alignmentDrive(
//            m_driveSubsystem,
//            () -> -controller.getLeftY(),
//            () -> -controller.getLeftX(),
//            () -> AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER)
//        )));
    spinUpTrigger.whileTrue(new AimbotCommand(m_armSubsystem, m_driveSubsystem, m_shooter, controller.getHID(), false));

//    shootTrigger.whileTrue(m_shooter.runShooterVelocity(true)
//        .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AUTO_AIM))
//        .alongWith(DriveCommands.alignmentDrive(
//            m_driveSubsystem,
//            () -> -controller.getLeftY(),
//            () -> -controller.getLeftX(),
//            () -> AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER)
//        )));

    shootTrigger.whileTrue(new AimbotCommand(m_armSubsystem, m_driveSubsystem, m_shooter, controller.getHID(), true));

    ampLineupTrigger.whileTrue(m_driveSubsystem.pathfollowFactory(FieldConstants.AMP_LINEUP)
        .finallyDo(() -> m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AMP).schedule()))
        .whileFalse(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.STOW));

    ampDepositeTrigger.whileTrue(Commands.runEnd(() -> m_shooter.setKickerPower(-0.5),
        () -> m_shooter.setKickerPower(0.0),
        m_shooter)
        .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AMP)));


    controller.leftBumper().whileTrue(
        m_shooter.runShooterVelocity(false, m_leftRPM.get(), m_rightRPM.get())
            .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.MANUAL_WRIST)));

    controller.rightBumper().whileTrue(
        m_shooter.runShooterVelocity(true, m_leftRPM.get(), m_rightRPM.get())
            .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.MANUAL_WRIST)));

    controller.pov(180).whileTrue(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.AMP));

    m_driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_driveSubsystem,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                            new Pose2d(m_driveSubsystem.getVisionPose().getTranslation(),
                            Rotation2d.fromDegrees(180.0)))),
                    m_driveSubsystem)
                .ignoringDisable(true));
  }

  /**
   * Use this method to configure any named commands needed for PathPlanner autos
   */
  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake", m_shooter.intakeCommand(0.75, 0.5, 0.1)
        .alongWith(m_armSubsystem.setDesiredStateFactory(ArmSubsystem.ArmState.INTAKE)));

    NamedCommands.registerCommand("AimAndShoot", new ShooterAutoCommand(m_armSubsystem, m_shooter, m_driveSubsystem));
  }

  private void configureDashboard() {
    ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");

    commandTab.add("Disable Arm Brake", m_armSubsystem.enableBrakeMode(false));
    commandTab.add("Enable Arm Brake", m_armSubsystem.enableBrakeMode(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonFactory.getSelectedAutonomous();
  }
}
