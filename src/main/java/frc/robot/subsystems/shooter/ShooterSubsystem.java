package frc.robot.subsystems.shooter;

import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import lib.logger.DataLogUtil;
import lib.logger.DataLogUtil.DataLogTable;
import org.littletonrobotics.junction.Logger;
import lib.utils.AimbotUtils;
import lib.utils.AllianceFlipUtil;
import lib.utils.FieldConstants;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs;

  private final Supplier<Pose2d> m_poseSupplier;

  private final DataLogTable m_logTable = DataLogUtil.getTable("Shooter");

  private double m_leftSpeedSetpoint = 3800.0;
  private double m_rightSpeedSetpoint = 3800.0;

  private final GosDoubleProperty m_leftPower = new
      GosDoubleProperty(false, "Shooter/Left RPM", 3600);
  private final GosDoubleProperty m_rightPower = new
      GosDoubleProperty(false, "Shooter/Right RPM", 3600);

  public ShooterSubsystem(ShooterIO io) {
    this(io, Pose2d::new);
  }

  public ShooterSubsystem(ShooterIO io, Supplier<Pose2d> poseSupplier) {
    m_io = io;
    m_inputs = new ShooterIOInputsAutoLogged();

    m_poseSupplier = poseSupplier;
    // turn on logging
    setupLogging();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);
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
    Pose3d speakerPose = new Pose3d(AllianceFlipUtil.apply(FieldConstants.CENTER_SPEAKER), new Rotation3d());
    Translation2d speakerPoseGround = speakerPose.getTranslation().toTranslation2d();
    double groundDistance = m_poseSupplier.get().getTranslation().getDistance(speakerPoseGround);

    m_leftSpeedSetpoint = AimbotUtils.getLeftSpeed(groundDistance);
    m_rightSpeedSetpoint = AimbotUtils.getRightSpeed(groundDistance);

    return runShooterVelocity(runKicker, m_leftSpeedSetpoint, m_rightSpeedSetpoint);
  }

  public Command runShooterVelocity(boolean runKicker, double leftRPM, double rightRPM) {
    return runEnd(() -> {
          m_leftSpeedSetpoint = leftRPM;
          m_rightSpeedSetpoint = rightRPM;

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
            setKickerPower(-0.1);
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

  public void setupLogging() {
    m_logTable.addDouble("LeftVelocity", () -> m_inputs.tlVelocityRPM, true);
    m_logTable.addDouble("RightVelocity", () -> m_inputs.trVelocityRPM, true);

    m_logTable.addDouble("LeftSetpoint", () -> m_leftSpeedSetpoint, true);
    m_logTable.addDouble("RightSetpoint", () -> m_rightSpeedSetpoint, true);

    m_logTable.addDouble("LeftAppliedVolts", () -> m_inputs.tlAppliedVolts, true);
    m_logTable.addDouble("RightAppliedVolts", () -> m_inputs.trAppliedVolts, true);

    m_logTable.addDouble("LeftCurrentDraw", () -> m_inputs.tlCurrentDraw, false);
    m_logTable.addDouble("RightCurrentDraw", () -> m_inputs.trCurrentDraw, false);

    m_logTable.addDouble("LeftTemperature", () -> m_inputs.tlTemperature, false);
    m_logTable.addDouble("RightTemperature", () -> m_inputs.trTemperature, false);

    m_logTable.addDouble("IntakeCurrent", () -> m_inputs.intakeCurrentDraw, true);
  }

  public boolean atSpeed() {
    return Math.abs(m_leftSpeedSetpoint - m_inputs.tlVelocityRPM) < 75
        && Math.abs(m_rightSpeedSetpoint - m_inputs.trVelocityRPM) < 75;
  }
}
