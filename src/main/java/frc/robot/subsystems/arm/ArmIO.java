package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double shoulderPositionRots = 0.0;
    public double wristPositionRots = 0.0;

    public double shoulerVelocityRotsPerSecond = 0.0;
    public double wristVelocityRotsPerSecond = 0.0;

    public double shoulderAppliedOutput = 0.0;
    public double wristAppliedOutput = 0.0;

    public double shoulderClosedLoopOutput = 0.0;
    public double wristClosedLoopOutput = 0.0;

    public double shoulderDesiredSetpoint = 0.0;
    public double wristDesiredSetpoint = 0.0;

    public double shoulderCurrentDraw = 0.0;
    public double wristCurrentDraw = 0.0;
  }

  default void updateInputs(ArmIOInputsAutoLogged inputs) {}

  default void setShoulderVoltage(double voltage) {}

  default void setShoulderAngle(double degrees, boolean useMM) {}

  default void setWristVoltage(double voltage) {}

  default void setWristAngle(double degrees, boolean useMM) {}
}
