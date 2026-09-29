/*
 * BMI088 error model for the simulated IMU, see imu_model.h
 */
#include "imu_model.h"

#include <algorithm>
#include <cmath>
#include <sstream>

using namespace crazyflie_interface;
using gz::math::Vector3d;

namespace
{
	constexpr double kTwoPi = 2.0 * M_PI;

	// Spin direction of m1..m4 (turningDirection in the SDF: ccw, cw, ccw, cw)
	constexpr std::array<double, 4> kSpin = {1.0, -1.0, 1.0, -1.0};

	void ReadSensor(const sdf::ElementPtr &_sdf, ImuErrorModel::SensorConfig &_s)
	{
		_s.noise_density = _sdf->Get<Vector3d>("noiseDensity", _s.noise_density).first;
		_s.bandwidth = _sdf->Get<double>("bandwidth", _s.bandwidth).first;
		_s.turn_on_bias_std = _sdf->Get<Vector3d>("turnOnBiasStd", _s.turn_on_bias_std).first;
		_s.bias_random_walk = _sdf->Get<double>("biasRandomWalk", _s.bias_random_walk).first;
		_s.scale_error_max = _sdf->Get<double>("scaleErrorMax", _s.scale_error_max).first;
		_s.misalignment_max = _sdf->Get<double>("misalignmentMax", _s.misalignment_max).first;
		_s.warmup_amplitude = _sdf->Get<double>("warmupAmplitude", _s.warmup_amplitude).first;
		_s.warmup_tau = _sdf->Get<double>("warmupTau", _s.warmup_tau).first;
	}
}

ImuErrorModel::Config ImuErrorModel::Config::FromSdf(const sdf::ElementPtr &_sdf)
{
	Config c;
	c.rate = _sdf->Get<double>("rate", c.rate).first;
	c.seed = _sdf->Get<uint32_t>("seed", c.seed).first;
	if (_sdf->HasElement("gyro"))
		ReadSensor(_sdf->GetElement("gyro"), c.gyro);
	if (_sdf->HasElement("accel"))
		ReadSensor(_sdf->GetElement("accel"), c.accel);
	if (_sdf->HasElement("vibration")) {
		const auto v = _sdf->GetElement("vibration");
		VibrationConfig &vc = c.vibration;
		vc.enabled = v->Get<bool>("enabled", vc.enabled).first;
		vc.ref_rot_velocity = v->Get<double>("refRotVelocity", vc.ref_rot_velocity).first;
		vc.accel_1x = v->Get<double>("accel1x", vc.accel_1x).first;
		vc.accel_2x = v->Get<double>("accel2x", vc.accel_2x).first;
		vc.gyro_1x = v->Get<double>("gyro1x", vc.gyro_1x).first;
		vc.gyro_2x = v->Get<double>("gyro2x", vc.gyro_2x).first;
		vc.z_ratio = v->Get<double>("zRatio", vc.z_ratio).first;
		vc.motor_spread = v->Get<double>("motorSpread", vc.motor_spread).first;
		vc.tau_up = v->Get<double>("tauUp", vc.tau_up).first;
		vc.tau_down = v->Get<double>("tauDown", vc.tau_down).first;
	}
	return c;
}

void ImuErrorModel::LowPass::Init(double _cutoff, double _rate)
{
	enabled = _cutoff > 0.0 && _cutoff < 0.5 * _rate;
	if (!enabled)
		return;
	const double k = std::tan(M_PI * _cutoff / _rate);
	const double norm = 1.0 / (1.0 + M_SQRT2 * k + k * k);
	b0 = k * k * norm;
	b1 = 2.0 * b0;
	b2 = b0;
	a1 = 2.0 * (k * k - 1.0) * norm;
	a2 = (1.0 - M_SQRT2 * k + k * k) * norm;
}

double ImuErrorModel::LowPass::Update(double _x)
{
	if (!enabled)
		return _x;
	if (!primed) {	// start in steady state, no step response on gravity
		x1 = x2 = y1 = y2 = _x;
		primed = true;
	}
	const double y = b0 * _x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
	x2 = x1;
	x1 = _x;
	y2 = y1;
	y1 = y;
	return y;
}

ImuErrorModel::ImuErrorModel(const Config &_config) :
	config_(_config),
	rng_(_config.seed)
{
	InitSensor(gyro_, config_.gyro);
	InitSensor(accel_, config_.accel);

	std::uniform_real_distribution<double> unit(-1.0, 1.0);
	std::uniform_real_distribution<double> angle(0.0, kTwoPi);
	for (size_t i = 0; i < 4; i++) {
		rotor_gain_[i] = 1.0 + config_.vibration.motor_spread * unit(rng_);
		for (double &phase : tone_phase_[i])
			phase = angle(rng_);
	}
}

void ImuErrorModel::InitSensor(Sensor &_sensor, const SensorConfig &_config)
{
	std::uniform_real_distribution<double> unit(-1.0, 1.0);
	_sensor.config = _config;

	const double m = _config.misalignment_max;
	const gz::math::Matrix3d misalignment(1.0, m * unit(rng_), m * unit(rng_),
										  m * unit(rng_), 1.0, m * unit(rng_),
										  m * unit(rng_), m * unit(rng_), 1.0);
	const double s = _config.scale_error_max;
	const gz::math::Matrix3d scale(1.0 + s * unit(rng_), 0.0, 0.0,
								   0.0, 1.0 + s * unit(rng_), 0.0,
								   0.0, 0.0, 1.0 + s * unit(rng_));
	_sensor.gain = misalignment * scale;

	for (int i = 0; i < 3; i++) {
		_sensor.bias[i] = _config.turn_on_bias_std[i] * normal_(rng_);
		_sensor.warmup[i] = _config.warmup_amplitude * unit(rng_);
		// White sequence with the datasheet density up to the Nyquist frequency
		_sensor.white_std[i] = _config.noise_density[i] * std::sqrt(0.5 * config_.rate);
		_sensor.filter[i].Init(_config.bandwidth, config_.rate);
	}
	_sensor.walk = Vector3d::Zero;
}

Vector3d ImuErrorModel::Output(Sensor &_sensor, const Vector3d &_true, double _dt, double _age)
{
	const Vector3d scaled = _sensor.gain * _true;
	const SensorConfig &c = _sensor.config;
	Vector3d out;
	for (int i = 0; i < 3; i++) {
		const double noisy = scaled[i] + _sensor.white_std[i] * normal_(rng_);
		_sensor.walk[i] += c.bias_random_walk * std::sqrt(_dt) * normal_(rng_);
		const double warmup = c.warmup_tau > 0.0 ? _sensor.warmup[i] * (1.0 - std::exp(-_age / c.warmup_tau)) : 0.0;
		out[i] = _sensor.filter[i].Update(noisy) + _sensor.bias[i] + _sensor.walk[i] + warmup;
	}
	return out;
}

double ImuErrorModel::FilterGain(const Sensor &_sensor, double _frequency) const
{
	// Magnitude of the sensor's second order Butterworth filter for a vibration tone
	if (!_sensor.filter[0].enabled)
		return 1.0;
	const double r = _frequency / _sensor.config.bandwidth;
	return 1.0 / std::sqrt(1.0 + r * r * r * r);
}

void ImuErrorModel::UpdateVibration(const std::array<double, 4> &_motor_omega, double _dt,
									Vector3d &_acc_vib, Vector3d &_gyro_vib)
{
	_acc_vib = Vector3d::Zero;
	_gyro_vib = Vector3d::Zero;
	const VibrationConfig &v = config_.vibration;
	if (!v.enabled)
		return;

	for (size_t i = 0; i < 4; i++) {
		// Rotor speed follows the command like the gz motor model (first order lag)
		const double tau = _motor_omega[i] > rotor_omega_[i] ? v.tau_up : v.tau_down;
		rotor_omega_[i] += (_motor_omega[i] - rotor_omega_[i]) * std::min(1.0, _dt / tau);
		rotor_phase_[i] = std::fmod(rotor_phase_[i] + kSpin[i] * rotor_omega_[i] * _dt, kTwoPi);

		const double r = rotor_omega_[i] / v.ref_rot_velocity;
		const double amplitude = r * r * rotor_gain_[i];
		const double f1 = rotor_omega_[i] / kTwoPi;
		const double p1 = rotor_phase_[i];
		const double p2 = 2.0 * rotor_phase_[i];
		const std::array<double, 8> &ph = tone_phase_[i];

		// Imbalance (1x) and blade pass (2x): rotating in the rotor plane, plus z
		const double a1 = amplitude * v.accel_1x * FilterGain(accel_, f1);
		const double a2 = amplitude * v.accel_2x * FilterGain(accel_, 2.0 * f1);
		_acc_vib += Vector3d(a1 * std::cos(p1 + ph[0]) + a2 * std::cos(p2 + ph[2]),
							 a1 * std::sin(p1 + ph[0]) + a2 * std::sin(p2 + ph[2]),
							 v.z_ratio * (a1 * std::cos(p1 + ph[1]) + a2 * std::cos(p2 + ph[3])));
		const double g1 = amplitude * v.gyro_1x * FilterGain(gyro_, f1);
		const double g2 = amplitude * v.gyro_2x * FilterGain(gyro_, 2.0 * f1);
		_gyro_vib += Vector3d(g1 * std::cos(p1 + ph[4]) + g2 * std::cos(p2 + ph[6]),
							  g1 * std::sin(p1 + ph[4]) + g2 * std::sin(p2 + ph[6]),
							  v.z_ratio * (g1 * std::cos(p1 + ph[5]) + g2 * std::cos(p2 + ph[7])));
	}
}

void ImuErrorModel::Apply(double _sim_time, const std::array<double, 4> &_motor_omega,
						  Vector3d &_acc, Vector3d &_gyro)
{
	if (start_time_ < 0.0)
		start_time_ = _sim_time;
	double dt = last_time_ >= 0.0 ? _sim_time - last_time_ : 1.0 / config_.rate;
	if (dt <= 0.0 || dt > 0.1)	// duplicate stamp or a jump (reset, pause)
		dt = 1.0 / config_.rate;
	last_time_ = _sim_time;

	Vector3d acc_vib, gyro_vib;
	UpdateVibration(_motor_omega, dt, acc_vib, gyro_vib);
	const double age = _sim_time - start_time_;
	_acc = Output(accel_, _acc, dt, age) + acc_vib;
	_gyro = Output(gyro_, _gyro, dt, age) + gyro_vib;
}

std::string ImuErrorModel::Describe() const
{
	std::ostringstream out;
	out.precision(3);
	const double deg = 180.0 / M_PI;
	out << "IMU model: gyro bias [" << gyro_.bias.X() * deg << ", " << gyro_.bias.Y() * deg << ", "
		<< gyro_.bias.Z() * deg << "] deg/s, gyro scale [" << gyro_.gain(0, 0) << ", " << gyro_.gain(1, 1)
		<< ", " << gyro_.gain(2, 2) << "], accel bias [" << accel_.bias.X() << ", " << accel_.bias.Y()
		<< ", " << accel_.bias.Z() << "] m/s^2, gyro filter " << config_.gyro.bandwidth
		<< " Hz, vibration " << (config_.vibration.enabled ? "on" : "off");
	return out.str();
}
