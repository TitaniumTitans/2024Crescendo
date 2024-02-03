package lib.properties.phoenix6;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.HeavyDoubleProperty;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

public final class Phoenix6PidPropertyBuilder {
   private final CoreTalonFX m_motor;
  private final SlotConfigs m_slot;
  private final boolean m_isConstant;
  private final String m_baseName;
  private final List<HeavyDoubleProperty> m_props;

  @SuppressWarnings("PMD.ImplicitSwitchFallThrough")
  public Phoenix6PidPropertyBuilder(String baseName, boolean isConstant, CoreTalonFX motor, int slot) {
    m_isConstant = isConstant;
    m_baseName = baseName;
    m_motor = motor;

    m_props = new ArrayList<>();

    switch (slot) {
      case 1:
        m_slot = SlotConfigs.from(new Slot1Configs());
        break;
      case 2:
        m_slot = SlotConfigs.from(new Slot2Configs());
        break;
      case 0:
      default:
        m_slot = SlotConfigs.from(new Slot0Configs());
    }
}

  private HeavyDoubleProperty createDoubleProperty(String propertyNameSuffix, double defaultValue, DoubleConsumer setter) {
    String propertyName = m_baseName + ".mm." + propertyNameSuffix;
    GosDoubleProperty prop = new GosDoubleProperty(m_isConstant, propertyName, defaultValue);

    return new HeavyDoubleProperty(setter, prop);
  }

  public Phoenix6PidPropertyBuilder addP(double defaultValue) {
    m_props.add(createDoubleProperty("kp", defaultValue, (double gain) -> {
      m_slot.kP = gain;
      m_motor.getConfigurator().apply(m_slot);
    }));
    return this;
  }

  public Phoenix6PidPropertyBuilder addI(double defaultValue) {
    m_props.add(createDoubleProperty("ki", defaultValue, (double gain) -> {
      m_slot.kI = gain;
      m_motor.getConfigurator().apply(m_slot);
    }));
    return this;
  }

  public Phoenix6PidPropertyBuilder addD(double defaultValue) {
    m_props.add(createDoubleProperty("kD", defaultValue, (double gain) -> {
      m_slot.kD = gain;
      m_motor.getConfigurator().apply(m_slot);
    }));
    return this;
  }

  public Phoenix6PidPropertyBuilder addKV(double defaultValue) {
    m_props.add(createDoubleProperty("kV", defaultValue, (double gain) -> {
      m_slot.kV = gain;
      m_motor.getConfigurator().apply(m_slot);
    }));
    return this;
  }

  public Phoenix6PidPropertyBuilder addKS(double defaultValue) {
    m_props.add(createDoubleProperty("kS", defaultValue, (double gain) -> {
      m_slot.kS = gain;
      m_motor.getConfigurator().apply(m_slot);
    }));
    return this;
  }

  public Phoenix6PidPropertyBuilder addKG(double defaultValue, GravityTypeValue gravityType) {
    m_props.add(createDoubleProperty("kG", defaultValue, (double gain) -> {
      m_slot.kG = gain;
      m_slot.GravityType = gravityType;
      m_motor.getConfigurator().apply(m_slot);
    }));
    return this;
  }

  public Phoenix6PidPropertyBuilder addKA(double defaultValue) {
    m_props.add(createDoubleProperty("kA", defaultValue, (double gain) -> {
      m_slot.kA = gain;
      m_motor.getConfigurator().apply(m_slot);
    }));
    return this;
  }

  public PidPropertyPublic build() {
    return new PidPropertyPublic(m_props);
  }
}
