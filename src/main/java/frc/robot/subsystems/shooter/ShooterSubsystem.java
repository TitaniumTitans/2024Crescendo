package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.utils.AimbotUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs;

  private double m_leftSpeedSetpoint = 3800.0;
  private double m_rightSpeedSetpoint = 3800.0;

  private final Supplier<Pose2d> m_poseSupplier;

  private final LoggedDashboardNumber m_leftSetpoint =
      new LoggedDashboardNumber("Shooter/Left RPM", 4000);
  private final LoggedDashboardNumber m_rightSetpoint =
      new LoggedDashboardNumber("Shooter/Right RPM", 4750);

  public ShooterSubsystem(ShooterIO io) {
    this(io, () -> new Pose2d());
  }

  public ShooterSubsystem(ShooterIO io, Supplier<Pose2d> pose2dSupplier) {
    m_io = io;
    m_inputs = new ShooterIOInputsAutoLogged();

    m_poseSupplier = pose2dSupplier;
    // turn on logging
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);

    Logger.recordOutput("Shooter/Has Note?", hasPiece());
  }

  public void setShooterPowerLeft(double power) {
    m_io.setMotorVoltageTL(power * 12.0);
    m_io.setMotorVoltageBL(power * 12.0);
  }

  public void setShooterPowerRight(double power) {
    m_io.setMotorVoltageTR(power * 12.0);
    m_io.setMotorVoltageBR(power * 12.0);
  }

  public void setKickerPower(double power) {
    m_io.setKickerVoltage(power * 12.0);
  }

  public void setIntakePower(double power) {
    m_io.setIntakeVoltage(power * 12.0);
  }

  public Command runShooterVelocity(boolean runKicker) {
    double distance = Units.metersToInches(AimbotUtils.getDistanceFromSpeaker(m_poseSupplier.get()));
    return runShooterVelocity(runKicker,
        () -> AimbotUtils.getLeftSpeed(distance),
        () -> AimbotUtils.getRightSpeed(distance));
  }

  public Command runShooterVelocity(boolean runKicker, DoubleSupplier leftRPM, DoubleSupplier rightRPM) {
    return runEnd(() -> {
          m_leftSpeedSetpoint = leftRPM.getAsDouble();
          m_rightSpeedSetpoint = rightRPM.getAsDouble();

          m_io.setLeftVelocityRpm(m_leftSpeedSetpoint);
          m_io.setRightVelocityRpm(m_rightSpeedSetpoint);

          if (runKicker) {
            m_io.setKickerVoltage(12.0);
            m_io.setIntakeVoltage(0.05);
          } else {
            m_io.setKickerVoltage(0.0);
            m_io.setIntakeVoltage(0.0);
          }
        },
        () -> {
          m_io.setMotorVoltageTL(0.0);
          m_io.setMotorVoltageTR(0.0);
          m_io.setKickerVoltage(0.0);
          m_io.setIntakeVoltage(0.0);
        });

  }

  public boolean hasPiece() {
    return m_io.hasPiece();
  }

  /** Command Factories */

  public Command intakeCommand(double intakePower, double kickerPower, double timeout) {
    Timer timer = new Timer();
    return runEnd(() -> {
          if (!hasPiece()) {
            setShooterPowerLeft(-0.1);
            setShooterPowerRight(-0.1);
            setIntakePower(intakePower);
            setKickerPower(kickerPower);
            timer.restart();
          } else if (!timer.hasElapsed(timeout)) {
            setKickerPower(-0.13);
            setIntakePower(0.0);
          } else {
            setShooterPowerRight(0.0);
            setShooterPowerLeft(0.0);
            setKickerPower(0.0);
            setIntakePower(0.0);
          }
        },
        () -> {
          setIntakePower(0.0);
          setKickerPower(0.0);
          setShooterPowerRight(0.0);
          setShooterPowerLeft(0.0);
        });
  }

  public Command setShooterPowerFactory(double left, double right, double intake) {
    return run(() -> {
      setShooterPowerLeft(left);
      setShooterPowerRight(right);
      setKickerPower(intake);
      setIntakePower(intake);
    });
  }

  public boolean atSpeed() {
    return Math.abs(m_leftSpeedSetpoint - m_inputs.tlVelocityRPM) < 75
        && Math.abs(m_rightSpeedSetpoint - m_inputs.trVelocityRPM) < 75;
  }
}
