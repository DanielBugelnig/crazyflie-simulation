#ifndef __CRTPUTILS_H___
#define __CRTPUTILS_H___

#include <iostream>
#include <cstdint>
#include <cmath>
#include "crtp.h"

#define CRTP_PORT_SETPOINT_SIM 0x09
#define CRTP_PORT_LOCALIZATION 0x06
#define CRTP_LOC_CHANNEL_EXT_POS 0x00
#define CRTP_LOC_CHANNEL_GEN_LOC 0x01
#define CRTP_GEN_LOC_ID_EXT_POS 0x08

// Conversion coefficient for Crazyflie
#define MAG_GAUSS_PER_LSB                                 666.7
#define SENSORS_DEG_PER_LSB_CFG                           ((2 * 2000.0) / 65536.0)
#define SENSORS_G_PER_LSB_CFG                             ((2 * 24) / 65536.0) // +-24 g, must match sensors_sitl.c

#define PI_CF 3.14159265359
#define DEG_TO_RAD_CF (PI_CF/180.0)
#define RAD_TO_DEG_CF (180.0/PI_CF)

#define GRAVITY_MAGNITUDE_CF (9.81) // we use the magnitude such that the sign/direction is explicit in calculations

// Motor model: PWM -> thrust per motor T(p) = a*p^2 + b*p with p = pwm/65535,
// then omega = sqrt(T / motorConstant) for the gz MulticopterMotorModel.
// The defaults are the Crazyflow linear model used by crazyflie_thrust_upgrade;
// the stock crazyflie model passes the firmware curve (powerDist.pwmToThrustA/B).
struct ThrustCurve {
	double a = 0.0;                    // N
	double b = 0.18;                   // N
	double motor_constant = 2.3375e-8; // N/(rad/s)^2, must match the SDF motor model
	double max_rot_velocity = 2797.0;  // rad/s, must match the SDF motor model
	uint16_t min_pwm = 7000;           // motors off below this command

	float omega(uint16_t pwm) const {
		if (pwm < min_pwm)
			return 0.0f;
		const double p = pwm / 65535.0;
		const double thrust = a * p * p + b * p;
		return static_cast<float>(std::fmin(std::sqrt(thrust / motor_constant), max_rot_velocity));
	}
};

// Physical value -> BMI088-style int16 register value (round to nearest, saturate)
static inline int16_t toSensorRaw(double value, double units_per_lsb) {
	const double raw = std::round(value / units_per_lsb);
	return static_cast<int16_t>(std::fmax(-32768.0, std::fmin(32767.0, raw)));
}

// Sensor type (first byte of crtp packet)
enum SensorTypeSim_e {
  SENSOR_GYRO_ACC_SIM           = 0,
  SENSOR_MAG_SIM                = 1,
  SENSOR_BARO_SIM               = 2,
  SENSOR_RANGE_SIM              = 3,
  SENSOR_BATT_SIM               = 4,
  SENSOR_TOF_SIM                = 5,
  SENSOR_FLOW_SIM               = 6,
  SENSOR_DECK_SIM               = 7,
  SENSOR_FLOW_RAW_SIM           = 8,
};

struct CrtpExtPose_s
{
  const crtp header;
  uint8_t id;
  float x; // in m
  float y; // in m
  float z; // in m
  float qx;
  float qy;
  float qz;
  float qw;
} __attribute__((packed));
CHECKSIZE_RESPONSE(CrtpExtPose_s)

struct CrtpExtPosition_s
{	
	const crtp header;
	float x; // in m
	float y; // in m
	float z; // in m
} __attribute__((packed));
CHECKSIZE_RESPONSE(CrtpExtPosition_s)

union Axis3i16 {
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
}__attribute__((packed));
CHECKSIZE_RESPONSE(Axis3i16)

union Axis3f {
	struct {
		float x;
		float y;
		float z;
	};
	float axis[3];
}__attribute__((packed));
CHECKSIZE_RESPONSE(Axis3f)

struct baro_s {
	const crtp header;
	uint8_t type;
	float pressure;           // mbar
	float temperature;        // degree Celcius
	float asl;                // m (ASL = altitude above sea level)
} __attribute__((packed));
CHECKSIZE_RESPONSE(baro_s)

struct imu_s {
	const crtp header;
	uint8_t type;
	union Axis3i16 acc;
	union Axis3i16 gyro;
} __attribute__((packed));
CHECKSIZE_RESPONSE(imu_s)

struct mag_s {
	const crtp header;
	uint8_t type;
	union Axis3i16 mag;
} __attribute__((packed));
CHECKSIZE_RESPONSE(mag_s)

// Deck stack bit flags, must match SITL_DECK_* in sensors_sitl.h
#define SIM_DECK_FLOW2        (1 << 0)
#define SIM_DECK_MULTIRANGER  (1 << 1)
#define SIM_DECK_AI           (1 << 2)

// VL53L1x distance (flow deck), processed like zranger2.c in the firmware
struct tof_s {
	const crtp header;
	uint8_t type;
	float distance;           // m
} __attribute__((packed));
CHECKSIZE_RESPONSE(tof_s)

// PMW3901 motion burst (flow deck), processed like flowdeck_v1v2.c in the firmware
struct flow_raw_s {
	const crtp header;
	uint8_t type;
	int16_t deltaX;           // counts, sensor axes
	int16_t deltaY;
	uint8_t squal;
	uint16_t shutter;
	uint8_t motion;
	float dt;                 // s since the previous read
} __attribute__((packed));
CHECKSIZE_RESPONSE(flow_raw_s)

struct deck_s {
	const crtp header;
	uint8_t type;
	uint8_t decks;
} __attribute__((packed));
CHECKSIZE_RESPONSE(deck_s)

struct pos_s {
	const crtp header;
	uint8_t type;
	union Axis3f pos;
} __attribute__((packed));
CHECKSIZE_RESPONSE(pos_s)
#endif //__CRTPUTILS_H___