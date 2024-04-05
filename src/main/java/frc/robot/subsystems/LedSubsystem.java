package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.BooleanSupplier;

public class LedSubsystem extends SubsystemBase {
    private final CANdle m_candle;
    private static final int NUM_LED = 160 + 8;
    private final BooleanSupplier m_hasNote;
    private boolean m_prevHasNote;

    private final Animation m_empty = new RainbowAnimation(0.0, 1.0, NUM_LED);
    private final Animation m_full = new RainbowAnimation(1.0, 0.75, NUM_LED);

    public LedSubsystem(BooleanSupplier hasNote) {
        m_candle = new CANdle(ShooterConstants.LED_ID);
//        m_candle.animate(m_red);
        m_hasNote = hasNote;
        m_prevHasNote = m_hasNote.getAsBoolean();
    }
    @Override
    public void periodic() {
        if (m_hasNote.getAsBoolean() && m_prevHasNote) {
//            m_candle.setLEDs(0, 255, 0);
            m_candle.animate(m_full);
        } else if (!m_hasNote.getAsBoolean() && !m_prevHasNote) {
//            m_candle.setLEDs(255, 0, 0);
            m_candle.animate(m_empty);
        }

        if (m_hasNote.getAsBoolean() != m_prevHasNote) {
            m_prevHasNote = m_hasNote.getAsBoolean();
        }
    }
}
