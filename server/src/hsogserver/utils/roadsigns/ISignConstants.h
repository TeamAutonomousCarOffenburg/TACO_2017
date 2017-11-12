#pragma once

namespace taco
{
enum SignType
{
	/** An unknown or invalid sign. */
	Unknown = 255,

	/** StVO 102 "Kreuzung mit Vorfahrt von Rechts" */
	UnmarkedIntersection = 0,

	/** StVO 206 "Stop und Vorfahrt Gewaehren" */
	Stop = 1,

	/** StVO 314 "Parken" */
	ParkingArea = 2,

	/** StVO 301 "Vorfahrt" */
	HaveWay = 3,

	/** StVO 209-30 "Vorgeschriebene Fahrtrichtung - geradeaus" */
	AheadOnly = 4, // Not used in AADC 2017

	/** StVO 205 "Vorfahrt Gewaehren" */
	GiveWay = 5,

	/** StVO 350 "Fussgaengerueberweg" */
	Crosswalk = 6,

	/** StVO 215 "Kreisverkehr" */
	Roundabout = 7, // Not used in AADC 2017

	/** StVO 276 "Ueberholverbot fuer alle Fahrzeuge" */
	NoOvertaking = 8, // Not used in AADC 2017

	/** StVO 267 "Verbot der Einfahrt" */
	NoEntryVehicularTraffic = 9, // Not used in AADC 2017

	/** "Teststrecke A9" */
	TestCourseA9 = 10,

	/** StVO 220 "Einbahnstrasse" */
	OneWayStreet = 11, // Not used in AADC 2017

	/** StVO 123 "Arbeitsstelle*/
	RoadWorks = 12,

	/** "Geschwindigkeitsbegerenzung 50km/h" */
	KMH_50 = 13,

	/** "Geschwindigkeitsbegerenzung 100km/h" */
	KMH_100 = 14,
};
}
