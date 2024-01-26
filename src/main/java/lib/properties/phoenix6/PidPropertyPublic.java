package lib.properties.phoenix6;

import com.gos.lib.properties.HeavyDoubleProperty;

import java.util.List;

public class PidPropertyPublic {
    private final List<HeavyDoubleProperty> m_properties;

    public PidPropertyPublic(List<HeavyDoubleProperty> properties) {
        m_properties = properties;
    }

    public final void updateIfChanged() {
        updateIfChanged(false);
    }

    public final void updateIfChanged(boolean forceUpdate) {
        for (HeavyDoubleProperty property : m_properties) {
            property.updateIfChanged(forceUpdate);
        }
    }
}
