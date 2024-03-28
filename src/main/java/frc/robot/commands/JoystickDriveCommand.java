package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.commands.DriveCommands.setSensitivity;


public class JoystickDriveCommand extends Command {
  private static final double DEADBAND = 0.1;
  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  private Rotation2d m_headingGoal;
  private boolean omegaJoystick = false;

  private final Timer m_timer;

  public JoystickDriveCommand(DriveSubsystem driveSubsystem,
                              DoubleSupplier xSupplier,
                              DoubleSupplier ySupplier,
                              DoubleSupplier omegaSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    m_headingGoal = driveSubsystem.getGyroRotation();
    m_timer = new Timer();
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.driveSubsystem);
  }

  @Override
  public void initialize() {
    m_headingGoal = driveSubsystem.getGyroRotation();
  }

  @Override
  public void execute() {
    double xInput = setSensitivity(xSupplier.getAsDouble(), 0.25);
    double yInput = setSensitivity(ySupplier.getAsDouble(), 0.25);
    double omegaInput = setSensitivity(omegaSupplier.getAsDouble(), 0.0)
            * Constants.DriveConstants.TURNING_SPEED.getValue();

    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xInput, yInput), DEADBAND);
    Rotation2d linearDirection =
        new Rotation2d(xInput, yInput);
    double omega = MathUtil.applyDeadband(omegaInput, DEADBAND) * driveSubsystem.getMaxAngularSpeedRadPerSec();

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    double rotationOutput = driveSubsystem.alignToAngle(m_headingGoal);

    if (Math.abs(omegaInput) > DEADBAND) {
      rotationOutput = omega;
      if (!omegaJoystick) {
        omegaJoystick = true;
      }
      m_timer.restart();
    } else {
      if (omegaJoystick
      && m_timer.hasElapsed(0.5)) {
        omegaJoystick = false;
        m_headingGoal = driveSubsystem.getGyroRotation();
        m_timer.stop();
        m_timer.reset();
      } else if (omegaJoystick
      && !m_timer.hasElapsed(0.5)) {
        rotationOutput = 0.0;
      }
    }

    Rotation2d heading;

    // if red change heading goal
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      heading = driveSubsystem.getRotation().plus(Rotation2d.fromDegrees(180));
    } else {
      heading = driveSubsystem.getRotation();
    }

    // Convert to field relative speeds & send command
    driveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
            rotationOutput,
            heading));
  }
}
