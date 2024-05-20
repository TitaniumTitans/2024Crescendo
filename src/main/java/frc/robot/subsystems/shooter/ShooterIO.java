package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    protected double tlVelocityRPM = 0.0;
    protected double trVelocityRPM = 0.0;

    protected double tlAppliedVolts = 0.0;
    protected double trAppliedVolts = 0.0;
    protected double kickerAppliedVolts = 0.0;
    protected double indexerAppliedVolts = 0.0;
    protected double intakeAppliedVolts = 0.0;

    protected double tlCurrentDraw = 0.0;
    protected double trCurrentDraw = 0.0;
    protected double kickerCurrentDraw = 0.0;
    protected double intakeCurrentDraw = 0.0;
    protected double indexerCurrentDraw = 0.0;

    protected double tlTemperature = 0.0;
    protected double trTemperature = 0.0;
    protected double indexerTemperature = 0.0;
    protected double intakeTemperature = 0.0;

    protected double tofDistanceIn = 0.0;


    public double getTlVelocityRPM() {
      return tlVelocityRPM;
    }

    public void setTlVelocityRPM(double tlVelocityRPM) {
      this.tlVelocityRPM = tlVelocityRPM;
    }

    public double getTrVelocityRPM() {
      return trVelocityRPM;
    }

    public void setTrVelocityRPM(double trVelocityRPM) {
      this.trVelocityRPM = trVelocityRPM;
    }

    public double getTlAppliedVolts() {
      return tlAppliedVolts;
    }

    public void setTlAppliedVolts(double tlAppliedVolts) {
      this.tlAppliedVolts = tlAppliedVolts;
    }

    public double getTrAppliedVolts() {
      return trAppliedVolts;
    }

    public void setTrAppliedVolts(double trAppliedVolts) {
      this.trAppliedVolts = trAppliedVolts;
    }

    public double getKickerAppliedVolts() {
      return kickerAppliedVolts;
    }

    public void setKickerAppliedVolts(double kickerAppliedVolts) {
      this.kickerAppliedVolts = kickerAppliedVolts;
    }

    public void setIndexerAppliedVolts(double indexerAppliedVolts) {
      this.indexerAppliedVolts = indexerAppliedVolts;
    }

    public double getIndexerAppliedVolts(double indexerAppliedVolts) {
      return this.indexerAppliedVolts;
    }

    public double getTlCurrentDraw() {
      return tlCurrentDraw;
    }

    public void setTlCurrentDraw(double tlCurrentDraw) {
      this.tlCurrentDraw = tlCurrentDraw;
    }

    public double getTrCurrentDraw() {
      return trCurrentDraw;
    }

    public void setTrCurrentDraw(double trCurrentDraw) {
      this.trCurrentDraw = trCurrentDraw;
    }

    public double getKickerCurrentDraw() {
      return kickerCurrentDraw;
    }

    public void setKickerCurrentDraw(double kickerCurrentDraw) {
      this.kickerCurrentDraw = kickerCurrentDraw;
    }

    public double getTlTemperature() {
      return tlTemperature;
    }

    public void setTlTemperature(double tlTemperature) {
      this.tlTemperature = tlTemperature;
    }

    public double getTrTemperature() {
      return trTemperature;
    }

    public void setTrTemperature(double trTemperature) {
      this.trTemperature = trTemperature;
    }

    public double getIndexerAppliedVolts() {
      return indexerAppliedVolts;
    }

    public double getIntakeAppliedVolts() {
      return intakeAppliedVolts;
    }

    public void setIntakeAppliedVolts(double intakeAppliedVolts) {
      this.intakeAppliedVolts = intakeAppliedVolts;
    }

    public double getIntakeCurrentDraw() {
      return intakeCurrentDraw;
    }

    public void setIntakeCurrentDraw(double intakeCurrentDraw) {
      this.intakeCurrentDraw = intakeCurrentDraw;
    }

    public double getIndexerCurrentDraw() {
      return indexerCurrentDraw;
    }

    public void setIndexerCurrentDraw(double indexerCurrentDraw) {
      this.indexerCurrentDraw = indexerCurrentDraw;
    }

    public double getIndexerTemperature() {
      return indexerTemperature;
    }

    public void setIndexerTemperature(double indexerTemperature) {
      this.indexerTemperature = indexerTemperature;
    }

    public double getIntakeTemperature() {
      return intakeTemperature;
    }

    public void setIntakeTemperature(double intakeTemperature) {
      this.intakeTemperature = intakeTemperature;
    }

    public double getTofDistanceIn() {
      return tofDistanceIn;
    }

    public void setTofDistanceIn(double tofDistanceIn) {
      this.tofDistanceIn = tofDistanceIn;
    }
  }
  default void setMotorVoltageTL(double voltage) {}
  default void setMotorVoltageTR(double voltage) {}
  default void setMotorVoltageBL(double voltage) {}
  default void setMotorVoltageBR(double voltage) {}
  default void setKickerVoltage(double voltage) {}
  default void setIntakeVoltage(double voltage) {}
  default void setLeftVelocityRpm(double rpm) {}
  default void setRightVelocityRpm(double rpm) {}
  default boolean hasPiece() {
    return false;
  }
  default void updateInputs(ShooterIOInputsAutoLogged inputs) {}
  default void stop() {}

}
