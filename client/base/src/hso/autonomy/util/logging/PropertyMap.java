package hso.autonomy.util.logging;

import java.text.DecimalFormat;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.math3.geometry.Vector;

public class PropertyMap
{
	private final Map<String, Property> properties = new LinkedHashMap<>();

	public void log(String name, Object value)
	{
		String stringValue = stringify(value);

		Property property = properties.get(name);
		if (property == null) {
			property = new Property();
		}

		boolean modified = property.value == null || !property.value.equals(stringValue);
		property.setValueChanged(modified);
		property.updatedLastCycle = true;
		property.value = stringValue;
		properties.put(name, property);
	}

	private String stringify(Object value)
	{
		if (value instanceof Vector) {
			return ((Vector) value).toString(new DecimalFormat("#.##"));
		} else if (value instanceof Float || value instanceof Double) {
			return String.format("%.2f", value);
		} else {
			return String.format("%s", value);
		}
	}

	public Map<String, Property> getMap()
	{
		return Collections.unmodifiableMap(properties);
	}

	public void resetPropertyStates()
	{
		for (Property property : properties.values()) {
			property.valueChanged = false;
			property.updatedLastCycle = false;
		}
	}

	public void update()
	{
		for (Property property : properties.values()) {
			property.updatedLastCycle = false;
		}
	}

	public class Property
	{
		public String value;

		public boolean valueChanged;

		public boolean updatedLastCycle;

		private int consecutiveUnchanged;

		public void setValueChanged(boolean valueChanged)
		{
			if (valueChanged) {
				this.valueChanged = true;
				consecutiveUnchanged = 10;
			} else {
				if (consecutiveUnchanged <= 0) {
					this.valueChanged = false;
				} else {
					consecutiveUnchanged--;
				}
			}
		}
	}
}
