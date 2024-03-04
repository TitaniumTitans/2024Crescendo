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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
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
import lib.utils.AllianceFlipUtil;
import lib.utils.FieldConstants;

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
  private final SendableChooser<Command> autoChooser;

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
            new ModuleIOTalonFX(Constants.DriveConstants.BR_MOD_CONSTANTS));
        m_shooter = new ShooterSubsystem(new ShooterIOKraken());
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
        m_armSubsystem = new ArmSubsystem(new ArmIOSim(), m_driveSubsystem::getPose);
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

    // Set up auto routines
    autoChooser = new SendableChooser<>();

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
                m_driveSubsystem,
                m_driveSubsystem::runCharacterizationVolts,
                m_driveSubsystem::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
    // configure named commands for auto
    configureNamedCommands();
    configureDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger intakeTrigger = controller.rightTrigger().and(controller.leftTrigger().negate());
    Trigger spinUpTrigger = controller.leftTrigger().and(controller.rightTrigger().negate());
    Trigger shootTrigger = controller.leftTrigger().and(controller.rightTrigger());

    intakeTrigger.whileTrue(m_shooter.intakeCommand(0.90, 0.5, 0.25)
        .alongWith(m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.INTAKE)));

    spinUpTrigger.whileTrue(m_shooter.runShooterVelocity(false)
        .alongWith(m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM)));
    shootTrigger.whileTrue(m_shooter.runShooterVelocity(true)
        .alongWith(m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM)));

    controller.x().whileTrue(m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AMP));
    controller.y().whileTrue(Commands.runEnd(() -> m_shooter.setKickerPower(-0.5),
        () -> m_shooter.setKickerPower(0.0),
        m_shooter));

//    controller.leftTrigger().onTrue(m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM))
//        .whileTrue(DriveCommands.alignmentDrive(
//            m_driveSubsystem,
//            () -> -controller.getLeftY(),
//            () -> -controller.getLeftX(),
//            () -> AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER)
//        ));

//    controller.rightTrigger().whileTrue(m_shooter.intakeCommand(0.75, 0.25, 0.1));
//    controller.rightBumper().whileTrue(m_shooter.intakeCommand(0.0, -0.25, 0.1));

    controller.leftBumper().whileTrue(m_climber.setClimberPowerFactory(0.55));
    controller.rightBumper().whileTrue(m_climber.setClimberPowerFactory(-0.55));

    double centerDistance = 1.34 - Units.inchesToMeters(3.0);

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
                            new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(240.25), 5.55),
                            Rotation2d.fromDegrees(180.0)))),
                    m_driveSubsystem)
                .ignoringDisable(true));
  }

  /**
   * Use this method to configure any named commands needed for PathPlanner autos
   */
  private void configureNamedCommands() {
    NamedCommands.registerCommand("Run Intake", Commands.run(() -> m_shooter.setIntakePower(0.5)));
//    NamedCommands.registerCommand("Run Shooter", Commands.run(m_shooter::runShooterVelocity));
  }

  private void configureDashboard() {
    ShuffleboardTab commandTab = Shuffleboard.getTab("Commads");

    commandTab.add("Disable Arm Brake", m_armSubsystem.enableBrakeMode(false));
    commandTab.add("Enable Arm Brake", m_armSubsystem.enableBrakeMode(true));

    commandTab.add("Center Robot Pose", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(
                    new Pose2d(
                        FieldConstants.FIELD_LENGTH / 2.0,
                        FieldConstants.FIELD_WIDTH / 2.0,
                        new Rotation2d())),
            m_driveSubsystem)
        .ignoringDisable(true));

    commandTab.add("CenterToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(1.34, 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));

    double centerDistance = 1.34 - Units.inchesToMeters(3.0);

    commandTab.add("2InchesToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(2.0), 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));

    commandTab.add("6InchesToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(6.0), 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));

    commandTab.add("12InchesToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(12.0), 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));

    commandTab.add("24InchesToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(24.0), 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));

    commandTab.add("48InchesToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(48.0), 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));

    commandTab.add("96InchesToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(96.0), 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));

    commandTab.add("192InchesToSpeaker", Commands.runOnce(
            () ->
                m_driveSubsystem.setPose(AllianceFlipUtil.apply(
                    new Pose2d(new Translation2d(centerDistance + Units.inchesToMeters(192.0), 5.55),
                        Rotation2d.fromDegrees(180.0)))),
            m_driveSubsystem)
        .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
