package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ShooterIOInputs {}

    default void setShoulderVoltage(double voltage) {}

    default void setShoulderAngle(double degrees) {}

    default void setWristVoltage(double voltage) {}

    default void setWristAngle(double degrees) {}
}
