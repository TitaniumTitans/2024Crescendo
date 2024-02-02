package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double tlVelocityRads = 0.0;
    public double trVelocityRads = 0.0;
    public double blVelocityRads = 0.0;
    public double brVelocityRads = 0.0;
    public double tlAppliedVolts = 0.0;
    public double trAppliedVolts = 0.0;
    public double blAppliedVolts = 0.0;
    public double brAppliedVolts = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double tlCurrentDraw = 0.0;
    public double trCurrentDraw = 0.0;
    public double blCurrentDraw = 0.0;
    public double brCurrentDraw = 0.0;
    public double kickerCurrentDraw = 0.0;
    public double tlTemperature = 0.0;
    public double trTemperature = 0.0;
    public double blTemperature = 0.0;
    public double brTemperature = 0.0;



  }
  default void setMotorVoltageTL(double voltage) {}
  default void setMotorVoltageTR(double voltage) {}
  default void setMotorVoltageBL(double voltage) {}
  default void setMotorVoltageBR(double voltage) {}
  default void setKickerVoltage(double voltage) {}
  default void setIntakeVoltage(double voltage) {}
  default void setLeftVelocityRpm(double rpm) {}
  default void setRightVelocityRpm(double rpm) {}
  default void updateInputs(ShooterIOInputs inputs) {}

}
