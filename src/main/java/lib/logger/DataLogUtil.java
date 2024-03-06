package lib.logger;

import edu.wpi.first.math.geometry.Pose2d;
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
  private static final List<DataLogTable> logTables =  new ArrayList<>();

  public static DataLogTable getTable(String name) {
    for (DataLogTable table : logTables) {
      if (table.name == name) {
        return table;
      }
    }

    DataLogTable table = new DataLogTable(name);
    logTables.add(table);
    return table;
  }

  public static void updateTables() {
    for (DataLogTable table : logTables) {
      table.updateLogs();
    }
  }

  public static class DataLogTable {
    public final String name;
    private final NetworkTable m_loggingTable;

    private final List<DoubleLogger> m_doubleLogs = new ArrayList<>();
    private final List<DoubleArrayLogger> m_doubleArrayLogs = new ArrayList<>();
    private final List<BooleanLogger> m_booleanLogs = new ArrayList<>();

    public DataLogTable(String loggingTableName) {
      this(NetworkTableInstance.getDefault().getTable(loggingTableName));
    }

    public DataLogTable(NetworkTable loggingTable) {
      m_loggingTable = loggingTable;
      name = m_loggingTable.toString();
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

    public void addPose2d(String logName, Supplier<Pose2d> updateChecker, boolean updateNT) {
      addDoubleArray(logName + "/Translation",
          () -> new double[]{
              updateChecker.get().getX(),
              updateChecker.get().getY()
          },
          updateNT);
      addDouble(logName + "/Rotation",
          () -> updateChecker.get().getRotation().getDegrees(),
          updateNT);
    }

    public void addPose2dArray(String logName, Supplier<Pose2d[]> updateChecker, boolean updateNT) {
      for (int i = 0; i < updateChecker.get().length; i++) {
        int finalI = i;
        addPose2d(logName + "/" + i,
            () -> updateChecker.get()[finalI],
            updateNT);
      }
    }

    public void updateLogs() {
      for (BooleanLogger booleanLogger : m_booleanLogs) {
        booleanLogger.updateBooleanEntry();
      }
      for (DoubleLogger doubleLogger : m_doubleLogs) {
        doubleLogger.updateDoubleEntry();
      }
      for (DoubleArrayLogger doubleArrayLogger : m_doubleArrayLogs) {
        doubleArrayLogger.updateDoubleArrayEntry();
      }
    }
  }
}
