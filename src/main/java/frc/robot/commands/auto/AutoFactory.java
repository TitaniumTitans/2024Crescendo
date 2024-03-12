package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

public final class AutoFactory {

  public enum AutoModes {
    FOUR_NOTE("FourNoteSpeaker"),
    TWO_NOTE_LEFT_3("TwoNoteSpeakerLeft3"),
    TWO_NOTE_LEFT_4("TwoNoteSpeakerLeft4"),
    TWO_NOTE_MIDDLE_2("TwoNoteSpeakerMiddle2"),
    TWO_NOTE_MIDDLE_5("TwoNoteSpeakerMiddle5"),
    TWO_NOTE_MIDDLE_6("TwoNoteSpeakerMiddle6"),
    TWO_NOTE_RIGHT_6("TwoNoteSpeakerRight6"),
    TWO_NOTE_RIGHT_7("TwoNotesSpeakerRight7"),
    LEAVE_WING("LeaveWing"),
    PRELOAD_AND_LEAVE_WING("OneNoteSpeakerAndLeaveWing"),
    PRELOAD_AND_SHOOT("JustShoot");


    public final String m_modeName;

    AutoModes(String modeName) {
      m_modeName = modeName;
    }
  }

  private static final AutoModes DEFAULT_MODE = AutoModes.TWO_NOTE_RIGHT_6;

  private final LoggedDashboardChooser<AutoModes> m_autonChooser;

  private final EnumMap<AutoModes, Command> m_modes;

  public AutoFactory() {
    m_autonChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_modes = new EnumMap<>(new HashMap<>());

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
