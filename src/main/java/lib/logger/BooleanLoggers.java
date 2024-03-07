package lib.logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.function.BooleanSupplier;

public class BooleanLoggers {
  public static class BooleanLogger {
    private final NetworkTableEntry m_networkTableEntry;
    private final boolean m_updateNT;
    private final BooleanSupplier m_booleanSupplier;
    private final BooleanLogEntry m_logEntry;
    private boolean m_lastValue;

    public BooleanLogger(NetworkTableEntry networkTableEntry, BooleanSupplier booleanSupplier, boolean updateNT) {
      m_networkTableEntry = networkTableEntry;
      m_booleanSupplier = booleanSupplier;

      m_updateNT = updateNT;

      m_lastValue = m_booleanSupplier.getAsBoolean();
      m_logEntry = new BooleanLogEntry(DataLogManager.getLog(), m_networkTableEntry.getName());
    }

    public void updateBooleanEntry() {
      if (m_updateNT) {
        m_networkTableEntry.setBoolean(m_booleanSupplier.getAsBoolean());
      }

      if (m_booleanSupplier.getAsBoolean() != m_lastValue) {
        m_lastValue = m_booleanSupplier.getAsBoolean();
        m_logEntry.append(m_lastValue);
      }
    }
  }
}
