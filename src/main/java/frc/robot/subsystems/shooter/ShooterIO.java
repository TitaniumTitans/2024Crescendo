package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    protected double tlVelocityRPM = 0.0;
    protected double trVelocityRPM = 0.0;
    protected double blVelocityRPM = 0.0;
    protected double brVelocityRPM = 0.0;

    protected double tlAppliedVolts = 0.0;
    protected double trAppliedVolts = 0.0;
    protected double blAppliedVolts = 0.0;
    protected double brAppliedVolts = 0.0;
    protected double kickerAppliedVolts = 0.0;
    protected double indexerAppliedVolts = 0.0;
    protected double intakeAppliedVolts = 0.0;

    protected double tlCurrentDraw = 0.0;
    protected double trCurrentDraw = 0.0;
    protected double blCurrentDraw = 0.0;
    protected double brCurrentDraw = 0.0;
    protected double kickerCurrentDraw = 0.0;
    protected double intakeCurrentDraw = 0.0;
    protected double indexerCurrentDraw = 0.0;

    protected double tlTemperature = 0.0;
    protected double trTemperature = 0.0;
    protected double blTemperature = 0.0;
    protected double brTemperature = 0.0;
    protected double indexerTemperature = 0.0;
    protected double intakeTemperature = 0.0;


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

    public double getBlVelocityRPM() {
      return blVelocityRPM;
    }

    public void setBlVelocityRPM(double blVelocityRPM) {
      this.blVelocityRPM = blVelocityRPM;
    }

    public double getBrVelocityRPM() {
      return brVelocityRPM;
    }

    public void setBrVelocityRPM(double brVelocityRPM) {
      this.brVelocityRPM = brVelocityRPM;
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

    public double getBlAppliedVolts() {
      return blAppliedVolts;
    }

    public void setBlAppliedVolts(double blAppliedVolts) {
      this.blAppliedVolts = blAppliedVolts;
    }

    public double getBrAppliedVolts() {
      return brAppliedVolts;
    }

    public void setBrAppliedVolts(double brAppliedVolts) {
      this.brAppliedVolts = brAppliedVolts;
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

    public double getBlCurrentDraw() {
      return blCurrentDraw;
    }

    public void setBlCurrentDraw(double blCurrentDraw) {
      this.blCurrentDraw = blCurrentDraw;
    }

    public double getBrCurrentDraw() {
      return brCurrentDraw;
    }

    public void setBrCurrentDraw(double brCurrentDraw) {
      this.brCurrentDraw = brCurrentDraw;
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

    public double getBlTemperature() {
      return blTemperature;
    }

    public void setBlTemperature(double blTemperature) {
      this.blTemperature = blTemperature;
    }

    public double getBrTemperature() {
      return brTemperature;
    }

    public void setBrTemperature(double brTemperature) {
      this.brTemperature = brTemperature;
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
  default void stop() {}

}
