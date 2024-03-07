package frc.robot.subsystems.arm;


public interface ArmIO {
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

  default void updateInputs(ArmIOInputs inputs) {}

  default void setArmVoltage(double voltage) {}

  default void setArmAngle(double degrees, double velocity) {}

  default void setWristVoltage(double voltage) {}

  default void setWristAngle(double degrees, double velocity) {}

  default void resetPosition() {}
  default void stop() {}
  default void enableBrakeMode(boolean enabled) {}
}
