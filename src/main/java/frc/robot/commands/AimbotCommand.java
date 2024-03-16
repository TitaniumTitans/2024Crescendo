package frc.robot.commands;

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import lib.utils.AimbotUtils;
import lib.utils.FieldConstants;
import lib.utils.FieldRelativeAccel;
import lib.utils.FieldRelativeSpeed;
import org.littletonrobotics.junction.Logger;


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

  private static final double TOLERENCE_DEGREES = 10.0;

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

    m_smallController.enableContinuousInput(-180, 180);
    m_fastController.enableContinuousInput(-180, 180);

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

    // get the robots velocity and acceleration
    FieldRelativeSpeed fieldRelativeSpeed = m_driveSubsystem.getFieldRelativeVelocity();
    FieldRelativeAccel fieldRelativeAccel = m_driveSubsystem.getFieldRelativeAcceleration();

    // TODO make an actual equation for shot time based on distance
    double distance = AimbotUtils.getDistanceFromSpeaker(m_driveSubsystem.getVisionPose());
    double shotTime = 1.05;

    Translation2d target = FieldConstants.CENTER_SPEAKER.toTranslation2d();
    Translation2d movingTarget = new Translation2d();

    // loop over movement calcs to better adjust for acceleration
    if (true) {
      for (int i = 0; i < 5; i++) {
        double virtualGoalX = target.getX()
            + shotTime * (fieldRelativeSpeed.vx + fieldRelativeAccel.ax * ShooterConstants.ACCEL_COMP_FACTOR.getValue());
        double virtualGoalY = target.getY()
            + shotTime * (fieldRelativeSpeed.vy + fieldRelativeAccel.ay * ShooterConstants.ACCEL_COMP_FACTOR.getValue());

        movingTarget = new Translation2d(virtualGoalX, virtualGoalY);
      }
    } else {
      movingTarget = target;
    }

    // get our desired rotation and error from it
    double desiredRotationDegs =
        AimbotUtils.getDrivebaseAimingAngle(m_driveSubsystem.getVisionPose())
            .getDegrees();
    double error = Math.abs(desiredRotationDegs - m_driveSubsystem.getRotation().getDegrees());

    // if we're far from our setpoint, move faster
    double omega;// = m_smallController.calculate(m_driveSubsystem.getRotation().getDegrees(), desiredRotationDegs);;
    if (error > 5.0) {
      omega = m_fastController.calculate(m_driveSubsystem.getRotation().getDegrees(), desiredRotationDegs);
    } else {
      omega = m_smallController.calculate(m_driveSubsystem.getRotation().getDegrees(), desiredRotationDegs);
    }

    // add a feedforward component to compensate for horizontal movement
    Translation2d linearFieldVelocity = new Translation2d(fieldRelativeSpeed.vx, fieldRelativeSpeed.vy);
    Translation2d tangentalVelocity = linearFieldVelocity
        .rotateBy(Rotation2d.fromDegrees(-desiredRotationDegs).unaryMinus());
    double tangentalComponent = tangentalVelocity.getX();

    double x = -DriveCommands.setSensitivity(-m_driverController.getLeftY(), 0.25);
    double y = -DriveCommands.setSensitivity(-m_driverController.getLeftX(), 0.25);

    x = MathUtil.applyDeadband(x, 0.1);
    y = MathUtil.applyDeadband(y, 0.1);

    Rotation2d heading;

    // if red change heading goal
    if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      heading = m_driveSubsystem.getRotation();
    } else {
      heading = m_driveSubsystem.getRotation().plus(Rotation2d.fromDegrees(180));
    }

    Logger.recordOutput("Aimbot/Tangental Velocity",  tangentalComponent);

    // Convert to field relative speeds & send command
    m_driveSubsystem.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            x * Constants.DriveConstants.MAX_LINEAR_SPEED,
            y * Constants.DriveConstants.MAX_LINEAR_SPEED,
            omega + tangentalComponent,
            heading
    ));

    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.AUTO_AIM);
    m_shooterSubsystem.runShooterVelocity(m_runKicker).execute();

    // set shooter speeds and rumble controller
    if (m_shooterSubsystem.atSpeed() && error < 15.0) {
      m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    } else {
      m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopWithX();
    m_shooterSubsystem.setShooterPowerLeft(0.0);
    m_shooterSubsystem.setShooterPowerRight(0.0);
    m_shooterSubsystem.setKickerPower(0.0);
    m_armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
    m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }
}
