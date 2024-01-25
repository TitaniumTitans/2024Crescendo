package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {}
  default void setVelocityTL(double rpm) {}
  default void setVelocityTR(double rpm) {}
  default void setVelocityBL(double rpm) {}
  default void setVelocityBR(double rpm) {}
  default void setKickerVoltage(double voltage) {}
  default void setIntakeVoltage(double voltage) {}

}
