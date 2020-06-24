/*
 * NMEA.h
 *
 * Created: 07.09.2018 10:54:14
 *  Author: Lukasz
 */ 


#ifndef NMEA_H_
#define NMEA_H_

#include <stdint.h>

#define PRECISION_THRESHOLD 3

enum nmeaMsgID {UNKNOWN, GGA, GLL, GSA, GSV, RMC, VTG};
enum naviType {NA, SPS, DGPS, PPS};
enum precision {NOSIGNAL, BAD, WEAK, MODERATE, GOOD, EXCELLENT, IDEAL};

typedef struct
{
	/* Dane s¹ dok³adniejsze ni¿ typ float, mo¿na u¿yæ double dla wiêkszej dok³adnoœci */
	float latitude;
	char latitudeHemisphere;
	float longitude;
	char longitudeHemisphere;
	float altitude;
	float velocity;
	float course;
	enum naviType navigationType;
	uint8_t satellitesCount;
	enum precision hdop;
	float differentialGPSDataAge;
	uint16_t differentialReferenceStationID;
	uint32_t date;
	float utcTime;
} GPS_Data;

void NMEA_parseString(char* nmeaString, GPS_Data* gpsData);

#endif /* NMEA_H_ */
