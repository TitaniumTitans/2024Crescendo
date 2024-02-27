package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double armPositionDegs = 0.0;
    public double wristPositionDegs = 0.0;

    public double armVelocityDegsPerSecond = 0.0;
    public double wristVelocityDegsPerSecond = 0.0;

    public double armAppliedOutput = 0.0;
    public double wristAppliedOutput = 0.0;

    public double armClosedLoopOutput = 0.0;
    public double wristClosedLoopOutput = 0.0;

    public double armDesiredSetpoint = 0.0;
    public double wristDesiredSetpoint = 0.0;

    public double armCurrentDraw = 0.0;
    public double wristCurrentDraw = 0.0;
  }

  default void updateInputs(ArmIOInputsAutoLogged inputs) {}

  default void setArmVoltage(double voltage) {}

  default void setArmAngle(double degrees) {}

  default void setWristVoltage(double voltage) {}

  default void setWristAngle(double degrees, boolean track) {}

  default void resetPosition() {}
  default void stop() {}
  default void enableBrakeMode(boolean enabled) {}
}
