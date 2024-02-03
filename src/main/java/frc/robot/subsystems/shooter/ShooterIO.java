package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {}
  default void setMotorVoltageTL(double voltage) {}
  default void setMotorVoltageTR(double voltage) {}
  default void setMotorVoltageBL(double voltage) {}
  default void setMotorVoltageBR(double voltage) {}
  default void setKickerVoltage(double voltage) {}

}
