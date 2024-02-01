package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public double leftClimberPosition = 0.0;
        public double rightClimberPosition = 0.0;
        public double leftClimberVelocity = 0.0;
        public double rightClimberVelocity = 0.0;
        public double leftClimberCurrentDraw = 0.0;
        public double rightClimberCurrentDraw = 0.0;
        public double leftClimberCurrentSetpoint = 0.0;
        public double rightClimberCurrentSetpoint = 0.0;
        public double leftClimberAppliedOutput = 0.0;
        public double rightClimberAppliedOutput = 0.0;

    }

    default void setRightPosition(double degrees) {}
    default void setLeftPosition(double degrees) {}
}
