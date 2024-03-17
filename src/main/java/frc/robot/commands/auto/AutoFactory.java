package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.EnumMap;
import java.util.HashMap;

public final class AutoFactory {

  public enum AutoModes {
    FRONT_WING_1("FrontWing1"),
    FRONT_WING_2("FrontWing2"),
    FRONT_WING_3("FrontWing3"),
    FRONT_WING_3_CONTESTED_5("FrontWing3Contested5"),
    AMP_WING_3("AmpWing3"),
    AMP_WING_3_CONTESTED_5("AmpWing3Contested5"),
    AMP_WING_1_2_3("AmpWing123"),
    SOURCE_WING_1("SourceWing1"),
    SOURCE_CONTESTED_1("SourceContested1"),
    SOURCE_WING_1_CONTESTED_1("SourceWing1Contested1");

    public final String m_modeName;

    AutoModes(String modeName) {
      m_modeName = modeName;
    }
  }

  private static final AutoModes DEFAULT_MODE = AutoModes.FRONT_WING_1;

  private final LoggedDashboardChooser<AutoModes> m_autonChooser;

  private final HashMap<AutoModes, Command> m_modes;

  public AutoFactory() {
    m_autonChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_modes = new HashMap<>();

    for (AutoModes mode : AutoModes.values()) {
      if (mode == DEFAULT_MODE) {
        m_autonChooser.addDefaultOption(mode.m_modeName, mode);
      } else {
        m_autonChooser.addOption(mode.m_modeName, mode);
      }

      m_modes.put(mode, AutoBuilder.buildAuto(mode.m_modeName));
    }
  }

  public Command getSelectedAutonomous() {
    AutoModes mode = m_autonChooser.get();
    return m_modes.get(mode);
  }
}
