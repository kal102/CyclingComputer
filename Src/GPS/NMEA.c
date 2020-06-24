/*
 * NMEA.c
 *
 * Created: 07.09.2018 10:53:53
 *  Author: Lukasz
 */ 

#include "GPS/NMEA.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum nmeaMsgID NMEA_specifyMessageID(char* nmeaString)
{
	char nmeaMsgIDString[4];
	
	/* Przeskanuj ³añuch w poszukiwaniu ID komunikatu NMEA, zignoruj resztê a¿ do znaku koñca linii */
	sscanf(nmeaString, "$GP%3c,%*[^\n]", nmeaMsgIDString);
	/* Dopisz NULL na koñcu, aby mo¿na by³o poprawnie porównaæ ³añcuch */
	*(nmeaMsgIDString + 3) = '\0';
	if (strcmp(nmeaMsgIDString, "GGA") == 0)
		return GGA;
	else if(strcmp(nmeaMsgIDString, "GLL") == 0)
		return GLL;
	else if(strcmp(nmeaMsgIDString, "GSA") == 0)
		return GSA;
	else if(strcmp(nmeaMsgIDString, "GSV") == 0)
		return GSV;
	else if(strcmp(nmeaMsgIDString, "RMC") == 0)
		return RMC;
	else if(strcmp(nmeaMsgIDString, "VTG") == 0)
		return VTG;
	else
		return UNKNOWN;
}

static inline float NMEA_convertVelocity(float velocity)
{
	return 1.852f * velocity;	// Knots to km/h
}

static enum precision NMEA_specifyPrecision(float dop)
{
	if (dop == 0.0)
		return NOSIGNAL;
	else if (dop < 1.0)
		return IDEAL;
	else if (dop < 3.0)
		return EXCELLENT;
	else if (dop < 6.0)
		return GOOD;
	else if (dop < 8.0)
		return MODERATE;
	else if (dop < 20.0)
		return WEAK;
	else
		return BAD;	
}

static uint8_t NMEA_calculateCRC(char* nmeaString) /* Nie dzia³a poprawnie */
{
	char* p;
	uint8_t crc = 0;
	
	/* Suma XOR wszystkich bajtów pomiêdzy $ a * */
	for (p = (nmeaString + 1); *p != '*'; p++)
	{
		crc ^= *p;
	}
	return crc;
}

static void NMEA_parseGGA(char* nmeaString, GPS_Data* gpsData)
{
	char* p;
	float utcTime = 0, latitude = 0, longitude = 0, hdop = 0, altitude = 0, geoidalSeparation = 0, differentialGPSDataAge = 0;
	char ns = '?', we = '?';
	uint8_t navigationType = 0, satellitesCount = 0, crc = 0;
	uint16_t differentialReferenceStationID = 0;
	
	p = strchr(nmeaString, ',');
	utcTime = atof(p+1);
	
	p = strchr(p+1, ',');
	latitude = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		ns = '?';
	else
		ns = *(p+1);
	
	p = strchr(p+1, ',');
	longitude = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		we = '?';
	else
		we = *(p+1);
	
	p = strchr(p+1, ',');
	navigationType = atoi(p+1);
	
	p = strchr(p+1, ',');
	satellitesCount = atoi(p+1);
	
	p = strchr(p+1, ',');
	hdop = atof(p+1);
	
	p = strchr(p+1, ',');
	altitude = atof(p+1);
	/* Pomiñ jednostkê */
	p = strchr(p+1, ',');
	
	p = strchr(p+1, ',');
	geoidalSeparation = atof(p+1);
	/* Pomiñ jednostkê */
	p = strchr(p+1, ',');
	
	p = strchr(p+1, ',');
	differentialGPSDataAge = atof(p+1);

	p = strchr(p+1, ',');
	differentialReferenceStationID = atoi(p+1);
	
	p = strchr(p+1, '*');
	crc = strtol(p+1, NULL, 16);
	
	if (crc == NMEA_calculateCRC(nmeaString))
	{
		gpsData->utcTime = utcTime;
		gpsData->latitude = latitude;
		gpsData->latitudeHemisphere = ns;
		gpsData->longitude = longitude;
		gpsData->longitudeHemisphere = we;
		gpsData->navigationType = (enum naviType)navigationType;
		gpsData->satellitesCount = satellitesCount;
		gpsData->hdop = NMEA_specifyPrecision(hdop);
		gpsData->altitude = altitude + geoidalSeparation;
		gpsData->differentialGPSDataAge = differentialGPSDataAge;
		gpsData->differentialReferenceStationID = differentialReferenceStationID;
	}
}

static void NMEA_parseGLL(char* nmeaString, GPS_Data* gpsData)
{
	char* p;
	float latitude = 0, longitude = 0, utcTime = 0;
	char ns = '?', we = '?', status = '?';
	uint8_t crc = 0;
	
	/* IdŸ do pierwszego przecinka */
	p = strchr(nmeaString, ',');
	/* Przekonwertuj ³añcuch na liczbê typu float, funkcja atof zatrzyma siê po napotkaniu nastêpnego przecinka */
	latitude = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		ns = '?';
	else
		ns = *(p+1);
		
	p = strchr(p+1, ',');
	longitude = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		we = '?';
	else
		we = *(p+1);
		
	p = strchr(p+1, ',');
	utcTime = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		status = '?';
	else
		status = *(p+1);
		
	p = strchr(p+1, '*');
	/* Konwersja z ASCII HEX na typ unsigned int */
	crc = strtol(p+1, NULL, 16);
	
	if (status == 'A' && crc == NMEA_calculateCRC(nmeaString))
	{
		gpsData->latitude = latitude;
		gpsData->latitudeHemisphere = ns;
		gpsData->longitude = longitude;
		gpsData->longitudeHemisphere = we;
		gpsData->utcTime = utcTime;
	}
}

static void NMEA_parseRMC(char* nmeaString, GPS_Data* gpsData)
{
	char* p;
	float utcTime = 0, latitude = 0, longitude = 0, velocity = 0, course = 0;
	uint32_t date = 0;
	char ns = '?', we = '?', status = '?';
	uint8_t crc = 0;
	
	p = strchr(nmeaString, ',');
	utcTime = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		status = '?';
	else
		status = *(p+1);
		
	p = strchr(p+1, ',');
	latitude = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		ns = '?';
	else
		ns = *(p+1);
		
	p = strchr(p+1, ',');
	longitude = atof(p+1);
	
	p = strchr(p+1, ',');
	if (*(p+1) == ',')
		we = '?';
	else
		we = *(p+1);
		
	p = strchr(p+1, ',');
	velocity = atof(p+1);
	
	p = strchr(p+1, ',');
	course = atof(p+1);
	
	p = strchr(p+1, ',');
	date = atoi(p+1);
	
	p = strchr(p+1, '*');
	crc = strtol(p+1, NULL, 16);
	
	if (status == 'A' && crc == NMEA_calculateCRC(nmeaString))
	{
		gpsData->utcTime = utcTime;
		gpsData->latitude = latitude;
		gpsData->latitudeHemisphere = ns;
		gpsData->longitude = longitude;
		gpsData->longitudeHemisphere = we;
		gpsData->velocity = NMEA_convertVelocity(velocity);
		gpsData->course = course;
		gpsData->date  = date;
	}
}

void NMEA_parseString(char* nmeaString, GPS_Data* gpsData)
{
	enum nmeaMsgID msgID;
	
	if(*nmeaString == '$')
		msgID = NMEA_specifyMessageID(nmeaString);
	else
		msgID = UNKNOWN;
	
	switch(msgID)
	{
		/* Przepisywanie wartoœci z ³añcucha do struktury, w zale¿noœci od typu komunikatu NMEA */
		case GGA:
			NMEA_parseGGA(nmeaString, gpsData);
			break;
		case GLL:
			NMEA_parseGLL(nmeaString, gpsData);
			break;
		case GSA:
			/* Nie obs³ugiwane */
			break;
		case GSV:
			/* Nie obs³ugiwane */
			break;
		case RMC:
			NMEA_parseRMC(nmeaString, gpsData);
			break;
		case VTG:
			/* Nie obs³ugiwane */
			break;
		default:
			break;
	}
}
