package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double tlVelocity = 0.0;
    public double trVelocity = 0.0;
    public double blVelocity = 0.0;
    public double brVelocity = 0.0;
    public double tlAppliedOutput = 0.0;
    public double trAppliedOutput = 0.0;
    public double blAppliedOutput = 0.0;
    public double brAppliedOutput = 0.0;
    public double kickerAppliedOutput = 0.0;
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
  default void updateInputs(ShooterIOInputs inputs) {}

}
