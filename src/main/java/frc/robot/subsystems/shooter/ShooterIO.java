package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double topLeftVelocity = 0.0;
    public double topRightVelocity = 0.0;
    public double bottomLeftVelocity = 0.0;
    public double bottomRightVelocity = 0.0;
    public double appliedOutput = 0.0;
    public double currentDraw = 0.0;
    public double temperature = 0.0;

  }

  default void updateInput(ShooterIOInputs inputs) {}
  default void setMotorVoltageTL(double voltage) {}
  default void setMotorVoltageTR(double voltage) {}
  default void setMotorVoltageBL(double voltage) {}
  default void setMotorVoltageBR(double voltage) {}
  default void setKickerVoltage(double voltage) {}

}
