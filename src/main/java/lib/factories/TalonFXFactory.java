package lib.factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import java.text.CompactNumberFormat;
import java.util.ArrayList;

public class TalonFXFactory {

    //ArrayList of all TalonFX motors
    public static ArrayList<TalonFX> TalonFxMotors = new ArrayList<TalonFX>();

    //This allows for the creation of a new Talon Brushless Motor
    TalonFXFactory(int id, String CanbusName) {
        TalonFXConfiguration Config = new TalonFXConfiguration();
        TalonFX talonFX = new TalonFX(id, CanbusName);

        TalonFXConfigurator Configurator = talonFX.getConfigurator();
        talonFX.getConfigurator().apply(new TalonFXConfiguration());

        TalonFxMotors.add(talonFX);
    }
}
