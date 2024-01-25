package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public class ShooterIOInputs {
        public double tLAngularVelocity = 0.0;
        public double tRAngularVelocity = 0.0;
        public double bLAngularVelocity = 0.0;
        public double bRAngularVelocity = 0.0;
        public double kickerAngularVelocity = 0.0;

        public double tLAppliedInputs = 0.0;
        public double tRAppliedInputs = 0.0;
        public double bLAppliedInputs = 0.0;
        public double bRAppliedInputs = 0.0;
        public double kickerAppliedInputs = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs){}

    default void setVelocityTL(double rpm) {}
    default void setVelocityTR(double rpm) {}
    default void setVelocityBL(double rpm) {}
    default void setVelocityBR(double rpm) {}
    default void setKickerVoltage(double voltage) {}
    default void setIntakeVoltage(double voltage) {}

}
