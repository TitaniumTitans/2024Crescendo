package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import lib.logger.DataLogUtil;
import lib.logger.DataLogUtil.DataLogTable;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO m_io;
  private final ShooterIO.ShooterIOInputs m_inputs;

  private final DataLogTable m_logTable = DataLogUtil.getTable("Shooter/");

  public ShooterSubsystem(ShooterIO io) {
    m_io = io;
    m_inputs = new ShooterIO.ShooterIOInputs();

    // turn on logging
    setupLogging();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
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
    return runEnd(() -> {
          m_io.setLeftVelocityRpm(0.0);
          m_io.setRightVelocityRpm(0.0);

          if (runKicker) {
            m_io.setKickerVoltage(3.0);
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
    return m_inputs.tofDistanceIn < 60;
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
            setShooterPowerLeft(0.0);
            setShooterPowerRight(0.0);
            setKickerPower(-0.25);
            setIntakePower(0.0);
          } else {
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

    m_logTable.addDouble("LeftAppliedVolts", () -> m_inputs.tlAppliedVolts, true);
    m_logTable.addDouble("RightAppliedVolts", () -> m_inputs.trAppliedVolts, true);

    m_logTable.addDouble("LeftCurrentDraw", () -> m_inputs.tlCurrentDraw, false);
    m_logTable.addDouble("RightCurrentDraw", () -> m_inputs.trCurrentDraw, false);

    m_logTable.addDouble("LeftTemperature", () -> m_inputs.tlTemperature, false);
    m_logTable.addDouble("RightTemperature", () -> m_inputs.trTemperature, false);
  }
}
