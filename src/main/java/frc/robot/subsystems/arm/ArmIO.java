package frc.robot.subsystems.arm;


import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    protected double armPositionDegs = 0.0;
    protected double wristPositionDegs = 0.0;

    protected double armVelocityDegsPerSecond = 0.0;
    protected double wristVelocityDegsPerSecond = 0.0;

    protected double armAppliedOutput = 0.0;
    protected double wristAppliedOutput = 0.0;

    protected double armClosedLoopOutput = 0.0;
    protected double wristClosedLoopOutput = 0.0;

    protected double armDesiredSetpoint = 0.0;
    protected double wristDesiredSetpoint = 0.0;

    protected double armCurrentDraw = 0.0;
    protected double wristCurrentDraw = 0.0;

    public double getArmPositionDegs() {
      return armPositionDegs;
    }

    public void setArmPositionDegs(double armPositionDegs) {
      this.armPositionDegs = armPositionDegs;
    }

    public double getWristPositionDegs() {
      return wristPositionDegs;
    }

    public void setWristPositionDegs(double wristPositionDegs) {
      this.wristPositionDegs = wristPositionDegs;
    }

    public double getArmVelocityDegsPerSecond() {
      return armVelocityDegsPerSecond;
    }

    public void setArmVelocityDegsPerSecond(double armVelocityDegsPerSecond) {
      this.armVelocityDegsPerSecond = armVelocityDegsPerSecond;
    }

    public double getWristVelocityDegsPerSecond() {
      return wristVelocityDegsPerSecond;
    }

    public void setWristVelocityDegsPerSecond(double wristVelocityDegsPerSecond) {
      this.wristVelocityDegsPerSecond = wristVelocityDegsPerSecond;
    }

    public double getArmAppliedOutput() {
      return armAppliedOutput;
    }

    public void setArmAppliedOutput(double armAppliedOutput) {
      this.armAppliedOutput = armAppliedOutput;
    }

    public double getWristAppliedOutput() {
      return wristAppliedOutput;
    }

    public void setWristAppliedOutput(double wristAppliedOutput) {
      this.wristAppliedOutput = wristAppliedOutput;
    }

    public double getArmClosedLoopOutput() {
      return armClosedLoopOutput;
    }

    public void setArmClosedLoopOutput(double armClosedLoopOutput) {
      this.armClosedLoopOutput = armClosedLoopOutput;
    }

    public double getWristClosedLoopOutput() {
      return wristClosedLoopOutput;
    }

    public void setWristClosedLoopOutput(double wristClosedLoopOutput) {
      this.wristClosedLoopOutput = wristClosedLoopOutput;
    }

    public double getArmDesiredSetpoint() {
      return armDesiredSetpoint;
    }

    public void setArmDesiredSetpoint(double armDesiredSetpoint) {
      this.armDesiredSetpoint = armDesiredSetpoint;
    }

    public double getWristDesiredSetpoint() {
      return wristDesiredSetpoint;
    }

    public void setWristDesiredSetpoint(double wristDesiredSetpoint) {
      this.wristDesiredSetpoint = wristDesiredSetpoint;
    }

    public double getArmCurrentDraw() {
      return armCurrentDraw;
    }

    public void setArmCurrentDraw(double armCurrentDraw) {
      this.armCurrentDraw = armCurrentDraw;
    }

    public double getWristCurrentDraw() {
      return wristCurrentDraw;
    }

    public void setWristCurrentDraw(double wristCurrentDraw) {
      this.wristCurrentDraw = wristCurrentDraw;
    }
  }

  default void updateInputs(ArmIOInputsAutoLogged inputs) {}

  default void setArmVoltage(double voltage) {}

  default void setArmAngle(double degrees, double velocity) {}

  default void setWristVoltage(double voltage) {}

  default void setWristAngle(double degrees, double velocity) {}

  default void resetPosition() {}
  default void stop() {}
  default void enableBrakeMode(boolean enabled) {}
}
