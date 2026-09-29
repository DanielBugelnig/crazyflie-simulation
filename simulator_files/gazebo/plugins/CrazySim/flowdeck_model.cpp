/*
 * Flow deck v2 sensor model, see flowdeck_model.h
 */
#include "flowdeck_model.h"

#include <algorithm>
#include <cmath>
#include <sstream>

using namespace crazyflie_interface;

namespace
{
	// Firmware flow model (mm_flow.c): Npix / thetapix model pixels per radian,
	// and the PMW3901 counts are 10x the model pixels (FLOW_RESOLUTION = 0.1)
	constexpr double kNpix = 35.0;
	constexpr double kThetaPix = 0.71674;
	constexpr double kFlowResolution = 0.1;
	constexpr double kCountsPerRad = kNpix / kThetaPix / kFlowResolution;

	// Motion register (pmw3901.h): 0xB0 has motionOccurred set, the driver only accepts this value
	constexpr uint8_t kMotionDetected = 0xB0;
	constexpr uint8_t kNoMotion = 0x30;

	// Below this cos(tilt) the floor is out of view
	constexpr double kMinCosTilt = 0.1;

	// VL53L1x: 27 deg field of view, sampled with 5 x 5 rays
	constexpr double kTofHalfFov = 13.5 * M_PI / 180.0;
	constexpr int kTofRaysPerAxis = 5;

	// VL53L1x standard deviation as assumed by the firmware (zranger2.c)
	double TofStdDev(double _distance)
	{
		constexpr double kPointA = 2.5, kStdA = 0.0025, kCoeff = 2.9214;
		return kStdA * (1.0 + std::exp(kCoeff * (_distance - kPointA)));
	}

	template <typename T>
	T ClampRound(double _value, double _low, double _high)
	{
		return static_cast<T>(std::clamp(std::round(_value), _low, _high));
	}
}

FlowDeckModel::Config FlowDeckModel::Config::FromSdf(const sdf::ElementPtr &_sdf)
{
	Config c;
	c.tof_rate = _sdf->Get<double>("tofRate", c.tof_rate).first;
	c.tof_max_range = _sdf->Get<double>("tofMaxRange", c.tof_max_range).first;
	c.tof_noise = _sdf->Get<bool>("tofNoise", c.tof_noise).first;
	c.tof_invalid_distance = _sdf->Get<double>("tofInvalidDistance", c.tof_invalid_distance).first;
	c.flow_rate = _sdf->Get<double>("flowRate", c.flow_rate).first;
	c.flow_noise = _sdf->Get<double>("flowNoise", c.flow_noise).first;
	c.scale_error_max = _sdf->Get<double>("flowScaleErrorMax", c.scale_error_max).first;
	c.yaw_misalignment_max = _sdf->Get<double>("flowYawMisalignmentMax", c.yaw_misalignment_max).first;
	c.min_height = _sdf->Get<double>("flowMinHeight", c.min_height).first;
	c.squal_mean = _sdf->Get<double>("squalMean", c.squal_mean).first;
	c.squal_std = _sdf->Get<double>("squalStd", c.squal_std).first;
	c.squal_threshold = _sdf->Get<double>("squalThreshold", c.squal_threshold).first;
	c.shutter_mean = _sdf->Get<double>("shutterMean", c.shutter_mean).first;
	c.shutter_std = _sdf->Get<double>("shutterStd", c.shutter_std).first;
	c.quality_tau = _sdf->Get<double>("qualityTau", c.quality_tau).first;
	c.seed = _sdf->Get<uint32_t>("seed", c.seed).first;
	return c;
}

FlowDeckModel::FlowDeckModel(const Config &_config) :
	config_(_config),
	rng_(_config.seed)
{
	// Per-drone mismatch between the sensor and the firmware model
	std::uniform_real_distribution<double> unit(-1.0, 1.0);
	scale_x_ = config_.scale_error_max * unit(rng_);
	scale_y_ = config_.scale_error_max * unit(rng_);
	yaw_misalignment_ = config_.yaw_misalignment_max * unit(rng_);

	// Start the quality processes in their stationary distribution
	squal_state_ = config_.squal_mean + config_.squal_std * normal_(rng_);
	shutter_state_ = config_.shutter_mean + config_.shutter_std * normal_(rng_);

	// VL53L1x cone around the body -z axis
	for (int i = 0; i < kTofRaysPerAxis; i++) {
		for (int j = 0; j < kTofRaysPerAxis; j++) {
			const double a = kTofHalfFov * (2.0 * i / (kTofRaysPerAxis - 1) - 1.0);
			const double b = kTofHalfFov * (2.0 * j / (kTofRaysPerAxis - 1) - 1.0);
			tof_rays_.push_back(gz::math::Vector3d(std::tan(a), std::tan(b), -1.0).Normalized());
		}
	}
}

void FlowDeckModel::SetSensorOffset(const gz::math::Vector3d &_offset)
{
	offset_ = _offset;
}

const gz::math::Vector3d &FlowDeckModel::SensorOffset() const
{
	return offset_;
}

bool FlowDeckModel::TofDue(double _sim_time)
{
	if (last_tof_time_ >= 0.0 && _sim_time - last_tof_time_ < 1.0 / config_.tof_rate - 1e-9)
		return false;
	last_tof_time_ = _sim_time;
	return true;
}

const std::vector<gz::math::Vector3d> &FlowDeckModel::TofRays() const
{
	return tof_rays_;
}

float FlowDeckModel::ProcessTof(std::vector<double> _axial_distances)
{
	if (_axial_distances.empty())
		return static_cast<float>(config_.tof_invalid_distance);

	// One distance for the cone: the median axial distance, i.e. the surface that
	// fills most of the cone (a flat floor gives the distance along the axis)
	auto median = _axial_distances.begin() + _axial_distances.size() / 2;
	std::nth_element(_axial_distances.begin(), median, _axial_distances.end());
	const double distance = *median;
	surface_offset_ = distance - flat_distance_;

	if (distance > config_.tof_max_range)
		return static_cast<float>(config_.tof_invalid_distance);
	double measured = distance;
	if (config_.tof_noise)
		measured += TofStdDev(distance) * normal_(rng_);
	return static_cast<float>(std::max(0.0, measured));
}

void FlowDeckModel::UpdateQuality(double _dt)
{
	// First order Gauss-Markov processes: texture patches pass by in bursts
	const double a = config_.quality_tau > 0.0 ? std::exp(-_dt / config_.quality_tau) : 0.0;
	const double b = std::sqrt(1.0 - a * a);
	squal_state_ = config_.squal_mean + a * (squal_state_ - config_.squal_mean)
		+ b * config_.squal_std * normal_(rng_);
	shutter_state_ = config_.shutter_mean + a * (shutter_state_ - config_.shutter_mean)
		+ b * config_.shutter_std * normal_(rng_);
}

std::optional<FlowDeckModel::Motion> FlowDeckModel::Step(double _sim_time, double _dt,
	const gz::math::Pose3d &_body_pose,
	const gz::math::Vector3d &_sensor_velocity_world,
	const gz::math::Vector3d &_angular_velocity_world)
{
	const gz::math::Quaterniond &rot = _body_pose.Rot();
	const double cos_tilt = rot.RotateVector(gz::math::Vector3d::UnitZ).Z();	// R[2][2]
	const double sensor_z = (_body_pose.Pos() + rot.RotateVector(offset_)).Z();

	// Height of the sensor above the surface below it (floor at z = 0 plus the
	// offset of the real surface measured by the ToF rays)
	flat_distance_ = cos_tilt > kMinCosTilt ? sensor_z / cos_tilt : 0.0;
	const double height = (flat_distance_ + surface_offset_) * cos_tilt;
	const bool floor_in_view = cos_tilt > kMinCosTilt && height > 1e-3;

	// Angular flow at the image centre, the model the firmware uses (mm_flow.c)
	if (floor_in_view) {
		const gz::math::Vector3d v = rot.RotateVectorReverse(_sensor_velocity_world);
		const gz::math::Vector3d w = rot.RotateVectorReverse(_angular_velocity_world);
		const double flow_x = (1.0 + scale_x_) * kCountsPerRad * (v.X() * cos_tilt / height - w.Y());
		const double flow_y = (1.0 + scale_y_) * kCountsPerRad * (v.Y() * cos_tilt / height + w.X());
		const double c = std::cos(yaw_misalignment_), s = std::sin(yaw_misalignment_);
		acc_x_ += (c * flow_x - s * flow_y) * _dt;
		acc_y_ += (s * flow_x + c * flow_y) * _dt;
	}

	if (!started_) {
		started_ = true;
		last_read_time_ = _sim_time;
		return std::nullopt;
	}
	const double read_dt = _sim_time - last_read_time_;
	if (read_dt < 1.0 / config_.flow_rate - 1e-9)
		return std::nullopt;
	last_read_time_ = _sim_time;

	UpdateQuality(read_dt);
	double squal = squal_state_;
	if (height < config_.min_height)
		squal *= std::max(0.0, height / config_.min_height);	// out of focus
	Motion m;
	m.squal = ClampRound<uint8_t>(squal, 0.0, 255.0);
	m.shutter = ClampRound<uint16_t>(shutter_state_, 0.0, 65535.0);
	m.dt = static_cast<float>(read_dt);

	if (!floor_in_view || height < config_.min_height || m.squal < config_.squal_threshold) {
		// Tracking lost: the motion of this frame is not measured
		acc_x_ = 0.0;
		acc_y_ = 0.0;
		m.delta_x = 0;
		m.delta_y = 0;
		m.motion = kNoMotion;
		return m;
	}

	// Per-read measurement noise, integer counts, the remainder carries over.
	// A read without counts has motionOccurred cleared, and the driver drops it.
	acc_x_ += config_.flow_noise * normal_(rng_);
	acc_y_ += config_.flow_noise * normal_(rng_);
	const double count_x = std::clamp(std::trunc(acc_x_), -32768.0, 32767.0);
	const double count_y = std::clamp(std::trunc(acc_y_), -32768.0, 32767.0);
	acc_x_ -= count_x;
	acc_y_ -= count_y;

	// Firmware axes -> sensor registers (inverse of the flip in the driver)
	m.delta_x = static_cast<int16_t>(-count_y);
	m.delta_y = static_cast<int16_t>(-count_x);
	m.motion = (count_x != 0.0 || count_y != 0.0) ? kMotionDetected : kNoMotion;
	return m;
}

std::string FlowDeckModel::Describe() const
{
	std::ostringstream out;
	out.precision(3);
	out << "flow deck: scale error x " << scale_x_ << " y " << scale_y_
		<< ", yaw misalignment " << yaw_misalignment_ * 180.0 / M_PI << " deg"
		<< ", noise " << config_.flow_noise << " counts"
		<< ", squal " << config_.squal_mean << " +- " << config_.squal_std
		<< " (threshold " << config_.squal_threshold << ")"
		<< ", ToF noise " << (config_.tof_noise ? "on" : "off")
		<< ", max range " << config_.tof_max_range << " m";
	return out.str();
}
