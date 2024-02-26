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

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOKraken;
import frc.robot.subsystems.arm.ArmIOPrototype;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.climber.ClimberIOPrototype;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon1;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;
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
  public final ArmSubsystem m_armSubsystem;
  private final ClimberSubsystem m_climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedDashboardNumber wristPower;
  private final LoggedDashboardNumber wristPosition;
  private final LoggedDashboardNumber armPower;
  private final LoggedDashboardNumber armPosition;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    Logger.recordOutput("Zero Pose3d", new Pose3d());

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
        m_armSubsystem = new ArmSubsystem(new ArmIOKraken());
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
        m_armSubsystem = new ArmSubsystem(new ArmIO() {});
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
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


    armPower = new LoggedDashboardNumber("Arm Power", 0.0);
    armPosition = new LoggedDashboardNumber("Arm Position", 0.0);
    wristPower = new LoggedDashboardNumber("Wrist Power", 0.0);
    wristPosition = new LoggedDashboardNumber("Wrist Position", 0.0);

    // Set up feedforward characterization
//    autoChooser.addOption(
//        "Drive FF Characterization",
//        new FeedForwardCharacterization(
//                m_driveSubsystem,
//                m_driveSubsystem::runCharacterizationVolts,
//                m_driveSubsystem::getCharacterizationVelocity));

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
//    controller.rightBumper().whileTrue(m_armSubsystem.setArmPowerFactory(0.15));
//    controller.leftBumper().whileTrue(m_armSubsystem.setArmPowerFactory(-0.15));
//
//    controller.rightTrigger().whileTrue(m_armSubsystem.setWristPowerFactory(0.15));
//    controller.leftTrigger().whileTrue(m_armSubsystem.setWristPowerFactory(-0.15));
//
//    controller.b().whileTrue(m_armSubsystem.setArmPositionFactory(180))
//        .whileFalse(m_armSubsystem.setArmPowerFactory(0.0));
//    controller.y().whileTrue(m_armSubsystem.setArmPositionFactory(90))
//        .whileFalse(m_armSubsystem.setArmPowerFactory(0.0));
//
//    controller.a().onTrue(m_armSubsystem.resetEncoderFactory());
//
//    m_driveSubsystem.setDefaultCommand(
//        DriveCommands.joystickDrive(
//            m_driveSubsystem,
//            () -> -controller.getLeftY(),
//            () -> -controller.getLeftX(),
//            () -> -controller.getRightX()));
//    controller.x().onTrue(Commands.runOnce(m_driveSubsystem::stopWithX, m_driveSubsystem));
//    controller
//        .start()
//        .onTrue(
//            Commands.runOnce(
//                    () ->
//                        m_driveSubsystem.setPose(
//                            new Pose2d(m_driveSubsystem.getPose().getTranslation(), new Rotation2d())),
//                    m_driveSubsystem)
//                .ignoringDisable(true));

    controller.x().whileTrue(m_shooter.setShooterPowerFactory(0.0, 0.0, 0.75))
        .whileFalse(m_shooter.setShooterPowerFactory(0.0, 0.0, 0.0));
    controller.y().whileTrue(m_shooter.setShooterPowerFactory(0.65, 0.6, 0.75))
        .whileFalse(m_shooter.setShooterPowerFactory(0.0, 0.0, 0.0));
  }

  /**
   * Use this method to configure any named commands needed for PathPlanner autos
   */
  private void configureNamedCommands() {
    NamedCommands.registerCommand("Run Intake", Commands.run(() -> m_shooter.setIntakePower(0.5)));
    NamedCommands.registerCommand("Run Shooter", Commands.run(m_shooter::runShooterVelocity));
  }

  private void configureDashboard() {
    ShuffleboardTab commandTab = Shuffleboard.getTab("Commads");

    commandTab.add("Disable Arm Brake", m_armSubsystem.enableBrakeMode(false));
    commandTab.add("Enable Arm Brake", m_armSubsystem.enableBrakeMode(true));
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
