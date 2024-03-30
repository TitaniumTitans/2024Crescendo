package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.BooleanSupplier;

public class LedSubsystem extends SubsystemBase {
    private final CANdle m_candle;
    private static final int m_numLed = 400;
    private final BooleanSupplier m_hasNote;

    private final Animation m_red = new ColorFlowAnimation(255,0, 0, 0, 0.5, m_numLed, ColorFlowAnimation.Direction.Forward);
    private final Animation m_blue = new ColorFlowAnimation(0, 255, 0, 0, 0.5, m_numLed, ColorFlowAnimation.Direction.Forward);

    public LedSubsystem(BooleanSupplier hasNote) {
        m_candle = new CANdle(ShooterConstants.LED_ID);
        m_candle.animate(m_red);
        m_hasNote = hasNote;
    }
    @Override
    public void periodic() {
        if (m_hasNote.getAsBoolean()) {
            m_candle.animate(m_red);}
        else {
            m_candle.animate(m_blue);}
    }

}
