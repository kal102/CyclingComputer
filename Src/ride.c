#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <ride.h>
#include "GPS/NMEA.h"

const uint8_t g = 10;
const int32_t DEGREE = 600000;

RIDE ride = {0};
RIDER rider = {.weight = 700};
BIKE bike = {.weight = 90, .type = ROAD};

static inline uint32_t min(uint32_t a, uint32_t b)
{
  return (a < b) ? a : b;
}

static inline uint32_t max(uint32_t a, uint32_t b)
{
  return (a > b) ? a : b;
}

static int32_t CoordinateConversion(float coordinate, char coordinateHemisphere)
{
	int32_t degrees, minutes;
	float integerPart;

	coordinate = coordinate / 100.0;
	minutes = (100.0 * modff(coordinate, &integerPart) / 60.0) * DEGREE;
	degrees = integerPart * DEGREE;

	degrees = degrees + minutes;
	if ((coordinateHemisphere == 'S') || (coordinateHemisphere == 'W'))
		degrees = -degrees;
	return degrees;
}

int32_t Diff (int32_t deg1, int32_t deg2)
{
  int32_t result = deg2 - deg1;
  if (result > (180 * DEGREE))
	  return result - 360 * DEGREE;
  else if (result < (-180 * DEGREE))
	  return result + 360 * DEGREE;
  else
	  return result;
}

uint16_t CosFix (int32_t angle)
{
  uint32_t u = labs(angle)>>16;
  u = (u * u * 6086)>>24;
  return 246 - u;
}

uint16_t DistanceBetween (int32_t lat1, int32_t long1, int32_t lat2, int32_t long2)
{
  int32_t dx = (Diff(long2, long1) * CosFix((lat1 + lat2)/2)) / 256;
  int32_t dy = Diff(lat2, lat1);
  uint32_t adx = labs(dx);
  uint32_t ady = labs(dy);
  uint32_t b = max(adx, ady);
  uint32_t a = min(adx, ady);
  if (b == 0)
	  return 0;
  return 95 * (b + (110 * a / b * a + 128) / 256) / 512;
}

uint16_t CourseTo (int32_t lat1, int32_t long1, int32_t lat2, int32_t long2)
{
  int16_t c;
  int32_t dx = (Diff(long2, long1) * CosFix((lat1 + lat2)/2)) / 256;
  int32_t dy = Diff(lat2, lat1);
  int32_t adx = labs(dx);
  int32_t ady = labs(dy);
  if (adx == 0)
	  c = 0;
  else if (adx < ady)
	  c = (adx * (45 + (16 * (ady - adx))/ady))/ady;
  else
	  c = 90 - (ady * (45 + (16 * (adx - ady))/adx))/adx;
  //
  if (dx <= 0 && dy < 0)
	  return c;
  else if (dx < 0 && dy >= 0)
	  return 180 - c;
  else if (dx >= 0 && dy >= 0)
	  return 180 + c;
  else
	  return 360 - c;
}

static inline uint32_t CalculateTimeInSeconds(struct tm time)
{
	return (time.tm_yday * 24UL * 3600UL + time.tm_hour * 3600UL + time.tm_min * 60UL + time.tm_sec);
}

static inline uint32_t CalculateAvgSpeed(uint32_t distance, struct tm time)
{
	uint32_t timeInSeconds;

	timeInSeconds = CalculateTimeInSeconds(ride.time);
	return (36 * ride.distance) / timeInSeconds;
}

static inline uint32_t CalculateAvgPower(uint32_t energy, struct tm time)
{
	uint32_t timeInSeconds;

	timeInSeconds = CalculateTimeInSeconds(ride.time);
	return ride.energy / timeInSeconds;
}

static inline uint32_t CalculateEnergy(uint32_t power, uint32_t time)
{
	return (uint32_t)((float)(power * time));
}

static inline uint32_t CalculateCalories(uint32_t energy)
{
	/* https://www.omnicalculator.com/sports/cycling-wattage#what-is-the-cycling-wattage */
	return (uint32_t)((float)energy / 4.18);
}

static inline float CalculateAirDensity(float altitude)
{
	return 1.225 * expf(-0.00011856 * altitude);
}

static inline float CalculateForceGravity(float alpha)
{
	float m, M;

	m = (float)rider.weight / 10.0;
	M = (float)bike.weight / 10.0;
	return (m + M) * g * sinf(alpha);
}

static inline float CalculateRollingResistance(float alpha, float crr)
{
	float m, M;

	m = (float)rider.weight / 10.0;
	M = (float)bike.weight / 10.0;
	return crr * (m + M) * g * cosf(alpha);
}

static inline float CalculateForceDrag(float v, float h, float cdA)
{
	float p;

	p = CalculateAirDensity(h);
	return 0.5 * cdA * p * powf(v, 2);
}

static uint16_t CalculateWattage(uint16_t speed, int16_t slope, int32_t altitude)
{
	float forceGrav, forceRoll, forceDrag;
	float v, h, p, alpha;
	float crr, cdA;
	const float loss = 0.04;

	v = (float)speed / 36.0;
	h = (float)altitude / 100.0;
	alpha = atanf((float)slope / 1000.0);

	/* Coefficient values and wattage calculation method taken from page:                */
	/* https://www.omnicalculator.com/sports/cycling-wattage#what-is-the-cycling-wattage */
	switch (bike.type)
	{
		case ROAD:
			crr = 0.005;
			cdA = 0.32;
			break;
		case TT:
			crr = 0.004;
			cdA = 0.29;
			break;
		case GRAVEL:
			crr = 0.007;
			cdA = 0.35;
			break;
		case CROSS:
			crr = 0.015;
			cdA = 0.35;
			break;
		case MOUNTAIN:
			crr = 0.020;
			cdA = 0.38;
			break;
		case FAT:
			crr = 0.032;
			cdA = 0.41;
			break;
		default:
			crr = 0.0;
			cdA = 0.0;
			break;
	}
	forceGrav = CalculateForceGravity(alpha);
	forceRoll = CalculateRollingResistance(alpha, crr);
	forceDrag = CalculateForceDrag(v, h, cdA);
	p = ((forceGrav + forceRoll + forceDrag) * v) / (1 - loss);

	if (p > 0)
		return (uint16_t)p;
	else
		return 0;
}

void Ride_ClearInfo(void)
{
	ride.state = STOPPED;
	ride.avgPower = 0;
	ride.avgSpeed = 0;
	ride.calories = 0;
	ride.distance = 0;
	ride.elevGain = 0;
	ride.power = 0;
	ride.slope = 0;
	ride.speed = 0;
	ride.time.tm_year = 0;
	ride.time.tm_mon = 0;
	ride.time.tm_mday = 1;
	ride.time.tm_hour = 0;
	ride.time.tm_min = 0;
	ride.time.tm_sec = 0;
}

enum RideState Ride_GetState(void)
{
	return ride.state;
}

void Ride_SetState(enum RideState state)
{
	ride.state = state;
}

const RIDE* Ride_GetInfo(void)
{
	return &ride;
}

void Ride_SetSettings(RIDER r,BIKE b)
{
	rider = r;
	bike = b;
}

RIDER Ride_GetRider(void)
{
	return rider;
}

BIKE Ride_GetBike(void)
{
	return bike;
}

void Ride_CalculateInfo(const GPS_Data *gpsData, const GPS_Data *lastGpsData, const struct bmp280_data *barometerData, const struct bmp280_data *lastBarometerData)
{
	int32_t latitude, longitude;
	int32_t lastLatitude, lastLongitude;
	uint16_t distDifference;
	int32_t elevDifference;

	ride.speed = (uint16_t)(gpsData->velocity * 10);

	if (ride.state != STARTED)
	{
		ride.slope = 0;
		ride.power = 0;
		return;
	}

	latitude = CoordinateConversion(gpsData->latitude, gpsData->latitudeHemisphere);
	longitude = CoordinateConversion(gpsData->longitude, gpsData->longitudeHemisphere);
	lastLatitude = CoordinateConversion(lastGpsData->latitude, lastGpsData->latitudeHemisphere);
	lastLongitude = CoordinateConversion(lastGpsData->longitude, lastGpsData->longitudeHemisphere);

	distDifference = DistanceBetween(lastLatitude, lastLongitude, latitude, longitude);
	elevDifference = barometerData->altitude - lastBarometerData->altitude;

	if (gpsData->velocity > AUTOPAUSE_VELOCITY)
	{
		if ((elevDifference < -ELEVATION_THRESHOLD) || (elevDifference > ELEVATION_THRESHOLD))
		{
			ride.slope = (10 * elevDifference)/(int32_t)distDifference;
		}
		else
		{
			ride.slope = 0;
		}
		ride.power = CalculateWattage(ride.speed, ride.slope, barometerData->altitude);
	}
	else
	{
		ride.slope = 0;
		ride.power = 0;
	}

	ride.distance += distDifference;
	if ((gpsData->velocity > AUTOPAUSE_VELOCITY) && (elevDifference > ELEVATION_THRESHOLD))
	{
		ride.elevGain += elevDifference;
	}

	ride.avgSpeed = CalculateAvgSpeed(ride.distance, ride.time);
	ride.energy  += CalculateEnergy(ride.power, 1);
	ride.calories = CalculateCalories(ride.energy);
	ride.avgPower = CalculateAvgPower(ride.energy, ride.time);
}

void Ride_TimerPulse(void)
{
	ride.time.tm_sec += 1;
	mktime(&(ride.time));
}
