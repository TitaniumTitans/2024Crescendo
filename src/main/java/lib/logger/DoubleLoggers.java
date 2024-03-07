package lib.logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DoubleLoggers {
  public static class DoubleLogger {
    private final NetworkTableEntry m_networkTableEntry;
    private final boolean m_updateNT;
    private final DoubleSupplier m_doubleSupplier;
    private final DoubleLogEntry m_logEntry;
    private double m_lastValue;

    public DoubleLogger(NetworkTableEntry networkTableEntry, DoubleSupplier doubleSupplier, boolean updateNT) {
      m_networkTableEntry = networkTableEntry;
      m_doubleSupplier = doubleSupplier;
      m_updateNT = updateNT;

      m_lastValue = m_doubleSupplier.getAsDouble();
      m_logEntry = new DoubleLogEntry(DataLogManager.getLog(), m_networkTableEntry.getName());
    }

    public void updateDoubleEntry() {
      if (m_updateNT) {
        m_networkTableEntry.setNumber(m_doubleSupplier.getAsDouble());
      }

      if (m_doubleSupplier.getAsDouble() != m_lastValue) {
        m_lastValue = m_doubleSupplier.getAsDouble();
        m_logEntry.append(m_lastValue);
      }
    }
  }

  public static class DoubleArrayLogger {
    private final NetworkTableEntry m_networkTableEntry;
    private final boolean m_updateNT;
    private final Supplier<double[]> m_doubleSupplier;
    private final DoubleArrayLogEntry m_logEntry;
    private double[] m_lastValue;

    public DoubleArrayLogger(NetworkTableEntry networkTableEntry, Supplier<double[]> doubleSupplier, boolean updateNT) {
      m_networkTableEntry = networkTableEntry;
      m_doubleSupplier = doubleSupplier;
      m_updateNT = updateNT;

      m_lastValue = m_doubleSupplier.get();
      m_logEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), m_networkTableEntry.getName());
    }

    public void updateDoubleArrayEntry() {
      if (m_updateNT) {
        m_networkTableEntry.setDoubleArray(m_doubleSupplier.get());
      }

      if (m_doubleSupplier.get() != m_lastValue) {
        m_lastValue = m_doubleSupplier.get();
        m_logEntry.append(m_lastValue);
      }
    }
  }
}
