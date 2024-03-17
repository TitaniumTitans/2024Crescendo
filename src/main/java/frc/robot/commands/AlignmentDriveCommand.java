package frc.robot.commands;

import com.pathplanner.lib.commands.PathfindHolonomic;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import lib.utils.AllianceFlipUtil;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class AlignmentDriveCommand extends Command {
  private static final double DEADBAND = 0.25;
  private final DriveSubsystem driveSubsystem;
  private final ArmSubsystem armSubsystem;
  private final Supplier<Pose2d> pointSupplier;
  private Command alignmentCommand;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaSupplier;

  private boolean reinit = false;

  public AlignmentDriveCommand(DriveSubsystem driveSubsystem,
                               ArmSubsystem armSubsystem,
                               DoubleSupplier xSupplier,
                               DoubleSupplier ySupplier,
                               DoubleSupplier thetaSupplier,
                               Supplier<Pose2d> point) {
    this.driveSubsystem = driveSubsystem;
    this.armSubsystem = armSubsystem;
    this.pointSupplier = point;

    this.xSupplier = xSupplier;
    this.ySupplier =ySupplier;
    this.thetaSupplier = thetaSupplier;

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.driveSubsystem, this.armSubsystem);
  }

  @Override
  public void initialize() {
    Pose2d point = AllianceFlipUtil.apply(pointSupplier.get());
    alignmentCommand = new PathfindHolonomic(
            point,
            DriveConstants.DEFAULT_CONSTRAINTS,
            0.0,
            driveSubsystem::getVisionPose,
            driveSubsystem::getRobotRelativeSpeeds,
            driveSubsystem::runVelocity,
            DriveConstants.HOLONOMIC_CONFIG,
            0.0,
            driveSubsystem
    );

    alignmentCommand.initialize();
  }

  @Override
  public void execute() {
    double xInput = DriveCommands.setSensitivity(xSupplier.getAsDouble(), 0.25);
    double yInput = DriveCommands.setSensitivity(ySupplier.getAsDouble(), 0.25);
    double omegaInput = DriveCommands.setSensitivity(thetaSupplier.getAsDouble(), 0.0) * 0.85;

    // if we have actual driver input
    if (xInput > DEADBAND
    || yInput > DEADBAND
    || omegaInput > DEADBAND) {
      // Apply deadband
      double linearMagnitude =
              MathUtil.applyDeadband(
                      Math.hypot(xInput, yInput), DEADBAND);
      Rotation2d linearDirection =
              new Rotation2d(xInput, yInput);
      double omega = MathUtil.applyDeadband(omegaInput, DEADBAND);

      // Calcaulate new linear velocity
      Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                      .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                      .getTranslation();

      Rotation2d heading;

      // if red change heading goal
      if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        heading = driveSubsystem.getRotation().plus(Rotation2d.fromDegrees(180));
      } else {
        heading = driveSubsystem.getRotation();
      }

      // Mark that we should re-check auto alignment
//      reinit = true;

      // Convert to field relative speeds & send command
      driveSubsystem.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                      omega * driveSubsystem.getMaxAngularSpeedRadPerSec(),
                      heading));
    } else {
      // if we've driven in the last loop cycle recheck auto alignment, and we're not too close to our goal
      if (reinit
      && driveSubsystem.getVisionPose().getTranslation().getDistance(pointSupplier.get().getTranslation())
          > Units.inchesToMeters(5.0)) {
        alignmentCommand.initialize();
      }

      alignmentCommand.execute();
    }

    // if we've finished auto aligning move arm to amp
    if (alignmentCommand.isFinished()) {
      armSubsystem.setDesiredState(ArmSubsystem.ArmState.AMP);
    }
  }

  @Override
  public boolean isFinished() {
    // finishing is handled by the "whileTrue" trigger method
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    alignmentCommand.end(interrupted);
    armSubsystem.setDesiredState(ArmSubsystem.ArmState.STOW);
  }
}
