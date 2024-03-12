package frc.robot.commands.auto;

public class AutoFactory {
  public enum AUTOS {
    ONE_PIECE_AMP_START("OneAmpStart"),
    THREE_PIECE_CENTER_START("ThreeCenterStart");

    AUTOS(String name) {
      this.name = name;
    }

    public final String name;
  }
}
