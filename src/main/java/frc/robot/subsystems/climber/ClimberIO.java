package frc.robot.subsystems.climber;

public interface ClimberIO {

    default void setLeftVoltage(double voltage) {}
    default void setRightVoltage(double voltage) {}
}
