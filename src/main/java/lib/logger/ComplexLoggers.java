package lib.logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Objects;
import java.util.function.Supplier;

public class ComplexLoggers {
  // quick and lazy loggers for complex datatypes
  public static class StringLogger {
    private final NetworkTableEntry m_networkTableEntry;
    private final boolean m_updateNT;
    private final Supplier<String> m_stringSupplier;
    private final StringLogEntry m_logEntry;
    private String m_lastValue;

    public StringLogger(NetworkTableEntry networkTableEntry, Supplier<String> stringSupplier, boolean updateNT) {
      m_networkTableEntry = networkTableEntry;
      m_stringSupplier = stringSupplier;
      m_updateNT = updateNT;

      m_lastValue = m_stringSupplier.get();
      m_logEntry = new StringLogEntry(DataLogManager.getLog(), m_networkTableEntry.getName());
    }

    public void updateStingEntry() {
      if (m_updateNT) {
        m_networkTableEntry.setString(m_stringSupplier.get());
      }

      if (!Objects.equals(m_stringSupplier.get(), m_lastValue)) {
        m_lastValue = m_stringSupplier.get();
        m_logEntry.append(m_lastValue);
      }
    }
  }
}
