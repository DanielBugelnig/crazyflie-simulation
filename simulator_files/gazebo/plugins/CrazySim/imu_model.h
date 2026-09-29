/*
 * BMI088 error model for the simulated IMU (gyro and accelerometer).
 *
 *   out = LPF(M S x_true + n) + vibration + b0 + b_rw(t) + b_warmup(t)
 *
 * n: white noise with the datasheet density, band limited by the sensor filter
 * (LPF); M S: misalignment and scale; b0: turn-on bias; b_rw: bias random walk;
 * b_warmup: first order drift after power-up. Vibration: rotor imbalance (1x)
 * and blade pass (2x) tones of the four motors, scaled by the sensor filter and
 * sampled at the IMU rate, so they alias like on the real chip. Per-drone values
 * are drawn once from the seed. Quantization happens in the plugin.
 */
#ifndef IMU_MODEL_H_
#define IMU_MODEL_H_

#include <array>
#include <cstdint>
#include <random>
#include <string>

#include <gz/math/Matrix3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>

namespace crazyflie_interface
{
	class ImuErrorModel
	{
		public: struct SensorConfig
		{
			gz::math::Vector3d noise_density;		// per axis [unit/sqrt(Hz)]
			double bandwidth = 0.0;					// sensor filter [Hz], 0: none
			gz::math::Vector3d turn_on_bias_std;	// per axis [unit]
			double bias_random_walk = 0.0;			// [unit/sqrt(s)]
			double scale_error_max = 0.0;			// uniform per axis
			double misalignment_max = 0.0;			// [rad], uniform per cross axis
			double warmup_amplitude = 0.0;			// [unit], uniform per axis
			double warmup_tau = 0.0;				// [s], 0: no warm-up drift
		};

		public: struct VibrationConfig
		{
			bool enabled = false;
			double ref_rot_velocity = 2000.0;	// rotor speed the amplitudes refer to [rad/s]
			double accel_1x = 0.0;				// per motor [m/s^2], in the rotor plane
			double accel_2x = 0.0;
			double gyro_1x = 0.0;				// per motor [rad/s]
			double gyro_2x = 0.0;
			double z_ratio = 0.0;				// z amplitude relative to the in-plane one
			double motor_spread = 0.0;			// relative amplitude spread between motors
			double tau_up = 0.0125;				// motor model time constants [s]
			double tau_down = 0.025;
		};

		public: struct Config
		{
			double rate = 1000.0;				// IMU samples per second
			SensorConfig gyro;
			SensorConfig accel;
			VibrationConfig vibration;
			uint32_t seed = 0;

			static Config FromSdf(const sdf::ElementPtr &_sdf);
		};

		public: explicit ImuErrorModel(const Config &_config);

		// One IMU sample. _acc/_gyro: true specific force [m/s^2] and angular
		// velocity [rad/s] in the body frame, replaced by the sensor output.
		// _motor_omega: commanded rotor speeds [rad/s], m1..m4.
		public: void Apply(double _sim_time, const std::array<double, 4> &_motor_omega,
						   gz::math::Vector3d &_acc, gz::math::Vector3d &_gyro);

		// Per-drone values, for the log
		public: std::string Describe() const;

		// Second order Butterworth low-pass (bilinear transform)
		private: struct LowPass
		{
			double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
			double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
			bool enabled = false;
			bool primed = false;
			void Init(double _cutoff, double _rate);
			double Update(double _x);
		};

		private: struct Sensor
		{
			SensorConfig config;
			gz::math::Matrix3d gain;			// M S
			gz::math::Vector3d bias;			// b0
			gz::math::Vector3d walk;			// b_rw
			gz::math::Vector3d warmup;			// final warm-up offset per axis
			gz::math::Vector3d white_std;		// per sample
			std::array<LowPass, 3> filter;
		};

		private: void InitSensor(Sensor &_sensor, const SensorConfig &_config);
		private: gz::math::Vector3d Output(Sensor &_sensor, const gz::math::Vector3d &_true, double _dt,
										   double _age);
		private: double FilterGain(const Sensor &_sensor, double _frequency) const;
		private: void UpdateVibration(const std::array<double, 4> &_motor_omega, double _dt,
									  gz::math::Vector3d &_acc_vib, gz::math::Vector3d &_gyro_vib);

		private: Config config_;
		private: Sensor gyro_;
		private: Sensor accel_;
		private: std::mt19937 rng_;
		private: std::normal_distribution<double> normal_{0.0, 1.0};
		private: double start_time_ = -1.0;
		private: double last_time_ = -1.0;

		// Rotors: filtered speed, phase and per-drone amplitude/phase draws
		private: std::array<double, 4> rotor_omega_{};
		private: std::array<double, 4> rotor_phase_{};
		private: std::array<double, 4> rotor_gain_{};
		private: std::array<std::array<double, 8>, 4> tone_phase_{};
	};
}

#endif
