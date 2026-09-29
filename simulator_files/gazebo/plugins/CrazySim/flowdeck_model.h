/*
 * Flow deck v2 sensor model: VL53L1x height sensor and PMW3901 optical flow.
 *
 * Produces what the chips report (distance, motion burst). The processing of
 * the real deck driver runs in the firmware (flowdeck_sitl.c, sensors_sitl.c).
 */
#ifndef FLOWDECK_MODEL_H_
#define FLOWDECK_MODEL_H_

#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>

namespace crazyflie_interface
{
	class FlowDeckModel
	{
		public: struct Config
		{
			// VL53L1x
			double tof_rate = 40.0;				// 25 ms timing budget [Hz]
			double tof_max_range = 3.0;			// medium distance mode [m]
			bool tof_noise = false;				// sigma(d) as assumed by zranger2.c
			double tof_invalid_distance = 8.19;	// sent without a valid return [m]
			// PMW3901
			double flow_rate = 100.0;			// reads per second (driver loop)
			double flow_noise = 1.0;			// count noise per read
			double scale_error_max = 0.0;		// per axis, drawn once per drone
			double yaw_misalignment_max = 0.0;	// [rad], drawn once per drone
			double min_height = 0.08;			// no tracking below [m]
			double squal_mean = 100.0;			// surface quality of the floor
			double squal_std = 0.0;
			double squal_threshold = 15.0;		// no motion reported below
			double shutter_mean = 500.0;
			double shutter_std = 0.0;
			double quality_tau = 0.3;			// correlation time of squal/shutter [s]
			uint32_t seed = 0;

			static Config FromSdf(const sdf::ElementPtr &_sdf);
		};

		// One read of the PMW3901 motion burst (sensor frame, as registers)
		public: struct Motion
		{
			int16_t delta_x;
			int16_t delta_y;
			uint8_t squal;
			uint16_t shutter;
			uint8_t motion;
			float dt;
		};

		public: explicit FlowDeckModel(const Config &_config);

		// Sensor position relative to the body origin, in the body frame
		public: void SetSensorOffset(const gz::math::Vector3d &_offset);
		public: const gz::math::Vector3d &SensorOffset() const;

		// Physics step with the true state; returns a motion burst when a read is due
		public: std::optional<Motion> Step(double _sim_time, double _dt,
										   const gz::math::Pose3d &_body_pose,
										   const gz::math::Vector3d &_sensor_velocity_world,
										   const gz::math::Vector3d &_angular_velocity_world);

		// VL53L1x: true when a measurement is due, the unit ray directions of its
		// 27 deg cone in the body frame, and the distance to report from the ray
		// hits projected onto the sensor axis (call after Step of the same time)
		public: bool TofDue(double _sim_time);
		public: const std::vector<gz::math::Vector3d> &TofRays() const;
		public: float ProcessTof(std::vector<double> _axial_distances);

		// Per-drone values, for the log
		public: std::string Describe() const;

		private: void UpdateQuality(double _dt);

		private: Config config_;
		private: gz::math::Vector3d offset_;
		private: std::vector<gz::math::Vector3d> tof_rays_;
		private: double scale_x_ = 0.0;
		private: double scale_y_ = 0.0;
		private: double yaw_misalignment_ = 0.0;

		// Flow accumulators in firmware axes (accpx, accpy) [counts]
		private: double acc_x_ = 0.0;
		private: double acc_y_ = 0.0;
		private: bool started_ = false;
		private: double last_read_time_ = 0.0;
		private: double last_tof_time_ = -1.0;
		private: double squal_state_ = 0.0;
		private: double shutter_state_ = 0.0;

		// Distance to a flat floor along the sensor axis, and the offset of the
		// real surface below the drone (obstacles) from the last ToF rays
		private: double flat_distance_ = 0.0;
		private: double surface_offset_ = 0.0;

		private: std::mt19937 rng_;
		private: std::normal_distribution<double> normal_{0.0, 1.0};
	};
}

#endif
