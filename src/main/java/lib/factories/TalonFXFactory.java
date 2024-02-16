package lib.factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import java.text.CompactNumberFormat;
import java.util.ArrayList;

public class TalonFXFactory {

    //ArrayList of all TalonFX motors. This should be accessable from all files.
    public static ArrayList<TalonFX> TalonFxMotors = new ArrayList<TalonFX>();

    //CONSTURCTOR: This allows for the creation of a new Talon Brushless Motor
    public TalonFXFactory(int id, String CanbusName) {
        TalonFXConfiguration Config = new TalonFXConfiguration();
        TalonFX talonFX = new TalonFX(id, CanbusName);

        TalonFXConfigurator Configurator = talonFX.getConfigurator();
        Configurator.apply(Config);

        TalonFxMotors.add(talonFX);
    }

    //CONSTURCTOR: This allows for the creation of a new Talon Brushless Motor with a preset Config
    public TalonFXFactory(int id, String CanbusName, TalonFXConfiguration Config) {
        TalonFX talonFX = new TalonFX(id, CanbusName);

        TalonFXConfigurator Configurator = talonFX.getConfigurator();
        Configurator.apply(Config);

        TalonFxMotors.add(talonFX);
    }

    //METHOD: Allows us to make edits to the config of a motor
    public void EditConfig(TalonFX talonFX, TalonFXConfiguration Config) {
        TalonFXConfigurator Configurator = talonFX.getConfigurator();
        Configurator.apply(Config);
    }
}
