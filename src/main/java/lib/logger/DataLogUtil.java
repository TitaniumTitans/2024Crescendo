package lib.logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lib.logger.DoubleLoggers.DoubleLogger;
import lib.logger.DoubleLoggers.DoubleArrayLogger;
import lib.logger.BooleanLoggers.BooleanLogger;

public class DataLogUtil {

  private final NetworkTable m_loggingTable;

  private final List<DoubleLogger> m_doubleLogs = new ArrayList<>();
  private final List<DoubleArrayLogger> m_doubleArrayLogs = new ArrayList<>();
  private final List<BooleanLogger> m_booleanLogs = new ArrayList<>();

  public DataLogUtil(String loggingTableName) {
    this(NetworkTableInstance.getDefault().getTable(loggingTableName));
  }

  public DataLogUtil(NetworkTable loggingTable) {
    m_loggingTable = loggingTable;
  }

  public void addDouble(String logName, DoubleSupplier updateChecker, boolean updateNt) {
    m_doubleLogs.add(new DoubleLogger(m_loggingTable.getEntry(logName), updateChecker, updateNt));
  }

  public void addDoubleArray(String logName, Supplier<double[]> updateChecker, boolean updateNt) {
    m_doubleArrayLogs.add(new DoubleArrayLogger(m_loggingTable.getEntry(logName), updateChecker, updateNt));
  }

  public void addBoolean(String logName, BooleanSupplier updateChecker, boolean updateNT) {
    m_booleanLogs.add(new BooleanLogger(m_loggingTable.getEntry(logName), updateChecker, updateNT));
  }

  public void updateLogs() {
    for (BooleanLogger booleanLogger: m_booleanLogs) {
      booleanLogger.updateBooleanEntry();
    }
    for (DoubleLogger doubleLogger: m_doubleLogs) {
      doubleLogger.updateDoubleEntry();
    }
    for (DoubleArrayLogger doubleArrayLogger: m_doubleArrayLogs) {
      doubleArrayLogger.updateDoubleArrayEntry();
    }
  }
}
