package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    protected double leftClimberPosition = 0.0;
    protected double rightClimberPosition = 0.0;

    protected double leftClimberVelocity = 0.0;
    protected double rightClimberVelocity = 0.0;

    protected double leftClimberCurrentDraw = 0.0;
    protected double rightClimberCurrentDraw = 0.0;

    protected double leftClimberCurrentSetpoint = 0.0;
    protected double rightClimberCurrentSetpoint = 0.0;

    protected double leftClimberAppliedOutput = 0.0;
    protected double rightClimberAppliedOutput = 0.0;

    public double getLeftClimberPosition() {
      return leftClimberPosition;
    }

    public void setLeftClimberPosition(double leftClimberPosition) {
      this.leftClimberPosition = leftClimberPosition;
    }

    public double getRightClimberPosition() {
      return rightClimberPosition;
    }

    public void setRightClimberPosition(double rightClimberPosition) {
      this.rightClimberPosition = rightClimberPosition;
    }

    public double getLeftClimberVelocity() {
      return leftClimberVelocity;
    }

    public void setLeftClimberVelocity(double leftClimberVelocity) {
      this.leftClimberVelocity = leftClimberVelocity;
    }

    public double getRightClimberVelocity() {
      return rightClimberVelocity;
    }

    public void setRightClimberVelocity(double rightClimberVelocity) {
      this.rightClimberVelocity = rightClimberVelocity;
    }

    public double getLeftClimberCurrentDraw() {
      return leftClimberCurrentDraw;
    }

    public void setLeftClimberCurrentDraw(double leftClimberCurrentDraw) {
      this.leftClimberCurrentDraw = leftClimberCurrentDraw;
    }

    public double getRightClimberCurrentDraw() {
      return rightClimberCurrentDraw;
    }

    public void setRightClimberCurrentDraw(double rightClimberCurrentDraw) {
      this.rightClimberCurrentDraw = rightClimberCurrentDraw;
    }

    public double getLeftClimberCurrentSetpoint() {
      return leftClimberCurrentSetpoint;
    }

    public void setLeftClimberCurrentSetpoint(double leftClimberCurrentSetpoint) {
      this.leftClimberCurrentSetpoint = leftClimberCurrentSetpoint;
    }

    public double getRightClimberCurrentSetpoint() {
      return rightClimberCurrentSetpoint;
    }

    public void setRightClimberCurrentSetpoint(double rightClimberCurrentSetpoint) {
      this.rightClimberCurrentSetpoint = rightClimberCurrentSetpoint;
    }

    public double getLeftClimberAppliedOutput() {
      return leftClimberAppliedOutput;
    }

    public void setLeftClimberAppliedOutput(double leftClimberAppliedOutput) {
      this.leftClimberAppliedOutput = leftClimberAppliedOutput;
    }

    public double getRightClimberAppliedOutput() {
      return rightClimberAppliedOutput;
    }

    public void setRightClimberAppliedOutput(double rightClimberAppliedOutput) {
      this.rightClimberAppliedOutput = rightClimberAppliedOutput;
    }
  }

  default void setRightVoltage(double volts) {}
  default void setLeftVoltage(double volts) {}
  default void setRightPosition(double degrees) {}
  default void setLeftPosition(double degrees) {}
  default void updateInputs(ClimberIOInputsAutoLogged inputs) {}
  default void stop() {}
  default void resetPosition() {}
}
