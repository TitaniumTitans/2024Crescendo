package frc.robot.subsystems.arm;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  public enum ArmStates {
    STOW,
    AIM,
    AIM_BLOCKED,
    AMP,
    TRAP,
    MOVING
  }

  private final ArmIO m_io;
  public ArmSubsystem(ArmIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(new ArmIOInputsAutoLogged());
  }

  public void setShoulderPower(double power) {
    m_io.setShoulderVoltage(power * 12.0);
  }

  public void setShoulderPosition(double degrees) {
    m_io.setShoulderAngle(degrees);
  }

  public void setWristPower(double power) {
    m_io.setWristVoltage(power * 12.0);
  }

  public void setWristPosition(double degrees) {
    m_io.setWristAngle(degrees);
  }

  public Translation2d getShooterTranslation(Rotation2d shoulderRotation, Rotation2d shooterRotation) {
    return Constants.ArmConstants.PIVOT_TRANSLATION_METERS.plus(
            new Translation2d(
                Constants.ArmConstants.SHOULDER_BAR_LENGTH_METERS,
                shoulderRotation))
        .plus(
            new Translation2d(
                Constants.ArmConstants.SHOOTER_BAR_LENGTH_METERS,
                shooterRotation)
        );
  }
}

