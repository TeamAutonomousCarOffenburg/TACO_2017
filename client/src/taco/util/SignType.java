package taco.util;

import com.google.gson.annotations.SerializedName;

@SuppressWarnings("SpellCheckingInspection")
public enum SignType {
	// TODO: make sure this is mapped to correctly
	UNKNOWN(-2),

	/** -1 muss gesendet werden, wenn ein Schild entfernt wurde*/
	@SerializedName("-1")
	SIGN_REMOVED(-1),

	/** StVO 102 "Kreuzung mit Vorfahrt von Rechts" */
	@SerializedName("0")
	UNMARKED_INTERSECTION(0),

	/** StVO 206 "Stop und Vorfahrt Gewaehren" */
	@SerializedName("1")
	STOP(1),

	/** StVO 314 "Parken" */
	@SerializedName("2")
	PARKING_AREA(2),

	/** StVO 301 "Vorfahrt" */
	@SerializedName("3")
	HAVE_WAY(3),

	/** StVO 209-30 "Vorgeschriebene Fahrtrichtung - geradeaus" */
	@SerializedName("4")
	AHEAD_ONLY(4), // Not used in AADC 2017

	/** StVO 205 "Vorfahrt Gewaehren" */
	@SerializedName("5")
	GIVE_WAY(5),

	/** StVO 350 "Fussgaengerueberweg" */
	@SerializedName("6")
	CROSSWALK(6),

	/** StVO 215 "Kreisverkehr" */
	@SerializedName("7")
	ROUNDABOUT(7), // Not used in AADC 2017

	/** StVO 276 "Ueberholverbot fuer alle Fahrzeuge" */
	@SerializedName("8")
	NO_OVERTAKING(8), // Not used in AADC 2017

	/** StVO 267 "Verbot der Einfahrt" */
	@SerializedName("9")
	NO_ENTRY_VEHICULAR_TRAFFIC(9), // Not used in AADC 2017

	/** "Teststrecke A9" */
	@SerializedName("10")
	TESTCOURSE_A9(10),

	/** StVO 220 "Einbahnstrasse" */
	@SerializedName("11")
	ONE_WAY_STREET(11), // Not used in AADC 2017

	/** StVO 123 "Arbeitsstelle*/
	@SerializedName("12")
	ROADWORKS(12),

	/** "Geschwindigkeitsbegerenzung 50km/h" */
	@SerializedName("13")
	KMH_50(13),

	/** "Geschwindigkeitsbegerenzung 100km/h" */
	@SerializedName("14")
	KMH_100(14);

	private final int value;

	SignType(int value)
	{
		this.value = value;
	}

	public int getValue()
	{
		return value;
	}
}
