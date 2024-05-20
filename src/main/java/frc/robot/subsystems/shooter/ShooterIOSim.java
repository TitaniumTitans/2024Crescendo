package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final FlywheelSim m_simLeft = new FlywheelSim(
      DCMotor.getFalcon500(1), 1, 0.001);
  private final FlywheelSim m_simRight = new FlywheelSim(
            DCMotor.getFalcon500(1), 1, 0.001);
  private final FlywheelSim m_simKicker = new FlywheelSim(
            DCMotor.getNeoVortex(1),1,0.001);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private double kickerAppliedVolts = 0.0;

  private final PIDController m_velocityController =
      new PIDController(Constants.ShooterConstants.SHOOTER_KP,
          Constants.ShooterConstants.SHOOTER_KI,
          Constants.ShooterConstants.SHOOTER_KD);

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(Constants.ShooterConstants.SHOOTER_KS,
          Constants.ShooterConstants.SHOOTER_KV);

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    m_simLeft.update(LOOP_PERIOD_SECS);
    m_simRight.update(LOOP_PERIOD_SECS);

    inputs.setTlVelocityRPM(m_simLeft.getAngularVelocityRPM());
    inputs.setTrVelocityRPM(m_simRight.getAngularVelocityRPM());

    inputs.setTlAppliedVolts(leftAppliedVolts);
    inputs.setTrAppliedVolts(rightAppliedVolts);
    inputs.setKickerAppliedVolts(kickerAppliedVolts);
  }

  @Override
  public void setRightVelocityRpm(double rpm) {
    // convert any RPM to RPS and then calculate ff and fb outputs
    this.setMotorVoltageTR(
        m_velocityController.calculate(m_simRight.getAngularVelocityRPM() / 60.0,
            rpm / 60.0) +
            m_feedforward.calculate(rpm / 60.0)
    );
  }

  @Override
  public void setLeftVelocityRpm(double rpm) {
    // convert any RPM to RPS and then calculate ff and fb outputs
    this.setMotorVoltageTL(
        m_velocityController.calculate(m_simLeft.getAngularVelocityRPM() / 60.0,
            rpm / 60.0) +
            m_feedforward.calculate(rpm / 60.0)
    );
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
