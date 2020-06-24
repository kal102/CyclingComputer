#ifndef RIDE_H
#define RIDE_H

#include <stdint.h>
#include <time.h>
#include "GPS/NMEA.h"
#include "BMP280/bmp280_user.h"

#define AUTOPAUSE_VELOCITY 3.0
#define AUTOSTART_VELOCITY 5.0
#define ELEVATION_THRESHOLD 10

enum RideState {STOPPED, STARTED, PAUSED};

typedef struct
{
	enum RideState state;
	uint16_t speed;
	uint16_t power;
	int16_t slope;
	uint32_t distance;
	struct tm time;
	uint32_t elevGain;
	uint16_t avgSpeed;
	uint16_t avgPower;
	uint32_t energy;
	uint32_t calories;
} RIDE;

typedef struct
{
	uint16_t weight;
} RIDER;

enum BikeType {ROAD = 0, TT = 1, GRAVEL = 2, CROSS = 3, MOUNTAIN = 4, FAT = 5};

typedef struct
{
	uint16_t weight;
	enum BikeType type;
} BIKE;

void Ride_ClearInfo(void);
enum RideState Ride_GetState(void);
void Ride_SetState(enum RideState state);
void Ride_SetSettings(RIDER r,BIKE b);
RIDER Ride_GetRider(void);
BIKE Ride_GetBike(void);
const RIDE * Ride_GetInfo(void);
void Ride_CalculateInfo(const GPS_Data *gpsData, const GPS_Data *lastGpsData, \
		const struct bmp280_data *barometerData, const struct bmp280_data *lastBarometerData);
void Ride_TimerPulse(void);

#endif
