package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.ShooterIO;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double shoulderPositionRads = 0.0;
        public double wristPositionRads = 0.0;
        public double shoulerVelocityRadsPerSecond = 0.0;
        public double wristVelocityRadsPerSecond = 0.0;
        public double shoulderAppliedOutput = 0.0;
        public double wristAppliedOutput = 0.0;
        public double shoulderDesiredSetpoint = 0.0;
        public double wristDesiredSetpoint = 0.0;
        public double shoulderCurrentDraw = 0.0;
        public double wristCurrentDraw = 0.0;
    }

    default void updateInputs(ArmIOInputsAutoLogged inputs) {}

    default void setShoulderVoltage(double voltage) {}

    default void setShoulderAngle(double degrees) {}

    default void setWristVoltage(double voltage) {}

    default void setWristAngle(double degrees) {}

    default Rotation2d getWristPosition() {return new Rotation2d();}

    default Rotation2d getShoulderPosition() {return new Rotation2d();}
}
