package frc.robot.subsystems.climber;

public interface ClimberIO {

    default void setRightPosition(double degrees) {}
    default void setLeftPosition(double degrees) {}
}
