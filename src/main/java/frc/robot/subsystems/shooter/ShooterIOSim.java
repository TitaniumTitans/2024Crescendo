package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO{
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final FlywheelSim m_simLeft = new FlywheelSim(
      DCMotor.getNeoVortex(1), 1, 0.001);
  private final FlywheelSim m_simRight = new FlywheelSim(
            DCMotor.getNeoVortex(1), 1, 0.001);
  private final FlywheelSim m_simKicker = new FlywheelSim(
            DCMotor.getNeoVortex(1),1,0.001);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private double kickerAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_simLeft.update(LOOP_PERIOD_SECS);
    m_simRight.update(LOOP_PERIOD_SECS);

    inputs.tlVelocityRots = m_simLeft.getAngularVelocityRadPerSec();
    inputs.trVelocityRots = m_simRight.getAngularVelocityRadPerSec();
    inputs.blVelocityRots = m_simLeft.getAngularVelocityRadPerSec();
    inputs.brVelocityRots = m_simRight.getAngularVelocityRadPerSec();

    inputs.tlAppliedVolts = leftAppliedVolts;
    inputs.trAppliedVolts = rightAppliedVolts;
    inputs.blAppliedVolts = leftAppliedVolts;
    inputs.brAppliedVolts = rightAppliedVolts;
    inputs.kickerAppliedVolts = kickerAppliedVolts;
  }

  @Override
  public void setMotorVoltageTL(double voltage) {
    leftAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    m_simLeft.setInputVoltage(voltage);
  }

  @Override
  public void setMotorVoltageTR(double voltage) {
    rightAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    m_simRight.setInputVoltage(voltage);
  }

  @Override
  public void setKickerVoltage(double voltage) {
    kickerAppliedVolts = MathUtil.clamp(voltage,-12.0,12.0);
    m_simKicker.setInputVoltage(voltage);
  }
}
