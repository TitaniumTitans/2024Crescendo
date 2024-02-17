package lib.factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;

import java.util.ArrayList;
import java.util.List;

public class TalonFXFactory {

  //ArrayList of all TalonFX motors. This should be accessable from all files.
  // Note: This should be private as we're only using it inside this class
  private static List<Pair<TalonFX, TalonFXConfiguration>> TalonFxMotors = new ArrayList<>();

  // Note: This is a static helper class, and as such shouldn't have an accessable constructor
  private TalonFXFactory() {
    throw new IllegalStateException("Helper class shouldn't be constructed");
  }

  //NOTE: This and the next method should return a talonFX, and both be static methods, not constructors
  //METHOD: This allows for the creation of a new Talon Brushless Motor
  public static TalonFX createTalon(int id) {
    TalonFX talonFX = new TalonFX(id, "canivore");
    talonFX.getConfigurator().apply(new TalonFXConfiguration());
    TalonFxMotors.add(new Pair<>(talonFX, new TalonFXConfiguration()));
    return talonFX;
  }

  public static TalonFX createTalon(int id, String canbusName) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFX talonFX = new TalonFX(id, canbusName);

    talonFX.getConfigurator().apply(config);

    TalonFxMotors.add(new Pair<>(talonFX, config));
    return talonFX;
  }

  //METHOD: This allows for the creation of a new Talon Brushless Motor with a preset Config
  public static TalonFX createTalon(int id, String canbusName, TalonFXConfiguration config) {
    TalonFX talonFX = new TalonFX(id, canbusName);

    talonFX.getConfigurator().apply(config);

    TalonFxMotors.add(new Pair<>(talonFX, config));
    return talonFX;
  }

  // METHOD: Checks each motor and handles sticky faults
  public static void handleFaults() {
    TalonFxMotors.forEach((Pair<TalonFX, TalonFXConfiguration> pair) -> {
      TalonFX talon = pair.getFirst();
      if (Boolean.TRUE.equals(talon.getStickyFault_BootDuringEnable().getValue())) {
        talon.getConfigurator().apply(pair.getSecond());
      }
    });
  }
}
