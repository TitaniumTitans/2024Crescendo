package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO{
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final FlywheelSim m_simTL = new FlywheelSim(
            DCMotor.getNeoVortex(1), 1, 1);
    private final FlywheelSim m_simTR = new FlywheelSim(
            DCMotor.getNeoVortex(1), 1, 1);
    private final FlywheelSim m_simBL = new FlywheelSim(
            DCMotor.getNeoVortex(1), 1, 1);
    private final FlywheelSim m_simBR = new FlywheelSim(
            DCMotor.getNeoVortex(1), 1, 1);
    private final FlywheelSim m_simKicker = new FlywheelSim(
            DCMotor.getNeoVortex(1),1,1);

    private double tLAppliedVolts = 0.0;
    private double tRAppliedVolts = 0.0;
    private double bLAppliedVolts = 0.0;
    private double bRAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

//    @Override
//    public void updateInputs(ShooterIOInputs inputs) {
//        m_simTL.update(LOOP_PERIOD_SECS);
//        m_simTR.update(LOOP_PERIOD_SECS);
//        m_simBL.update(LOOP_PERIOD_SECS);
//        m_simBR.update(LOOP_PERIOD_SECS);
//
//        inputs.tLAngularVelocity = m_simTL.getAngularVelocityRadPerSec();
//        inputs.tRAngularVelocity = m_simTR.getAngularVelocityRadPerSec();
//        inputs.bLAngularVelocity = m_simBL.getAngularVelocityRadPerSec();
//        inputs.bRAngularVelocity = m_simBR.getAngularVelocityRadPerSec();
//        inputs.kickerAngularVelocity = m_simKicker.getAngularVelocityRadPerSec();
//
//        inputs.tLAppliedInputs = tLAppliedVolts;
//        inputs.tRAppliedInputs = tRAppliedVolts;
//        inputs.bLAppliedInputs = bLAppliedVolts;
//        inputs.bRAppliedInputs = bRAppliedVolts;
//        inputs.kickerAppliedInputs = kickerAppliedVolts;
//    }

    @Override
    public void setMotorVoltageTL(double voltage) {
        tLAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        m_simTL.setInputVoltage(voltage);
    }

    @Override
    public void setMotorVoltageTR(double voltage) {
        tRAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        m_simTR.setInputVoltage(voltage);
    }

    @Override
    public void setMotorVoltageBL(double voltage) {
        bLAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        m_simBL.setInputVoltage(voltage);
    }

    @Override
    public void setMotorVoltageBR(double voltage) {
        bRAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        m_simBR.setInputVoltage(voltage);
    }

    @Override
    public void setKickerVoltage(double voltage) {
        kickerAppliedVolts = MathUtil.clamp(voltage,-12.0,12.0);
        m_simKicker.setInputVoltage(voltage);
    }
}
