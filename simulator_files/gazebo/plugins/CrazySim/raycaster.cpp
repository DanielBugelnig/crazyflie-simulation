/*
 * Ray casting against the collision shapes of the world, see raycaster.h
 */
#include "raycaster.h"

#include <algorithm>
#include <cmath>

#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Static.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

using namespace crazyflie_interface;
using gz::math::Vector3d;

namespace
{
	constexpr double kInf = std::numeric_limits<double>::infinity();
	constexpr double kEps = 1e-12;

	// Smallest t >= 0 of the ray o + t d with a sphere at c; 0 when o is inside
	double RaySphere(const Vector3d &_o, const Vector3d &_d, const Vector3d &_c, double _r)
	{
		const Vector3d oc = _o - _c;
		const double b = oc.Dot(_d);
		const double c = oc.Dot(oc) - _r * _r;
		if (c <= 0.0)
			return 0.0;
		const double disc = b * b - c;
		if (disc < 0.0)
			return kInf;
		const double t = -b - std::sqrt(disc);
		return t >= 0.0 ? t : kInf;
	}

	// Side surface of a z aligned cylinder, only where |z| <= _half_length
	double RayCylinderSide(const Vector3d &_o, const Vector3d &_d, double _r, double _half_length)
	{
		const double a = _d.X() * _d.X() + _d.Y() * _d.Y();
		if (a < kEps)
			return kInf;
		const double b = _o.X() * _d.X() + _o.Y() * _d.Y();
		const double c = _o.X() * _o.X() + _o.Y() * _o.Y() - _r * _r;
		const double disc = b * b - a * c;
		if (disc < 0.0)
			return kInf;
		for (double t : {(-b - std::sqrt(disc)) / a, (-b + std::sqrt(disc)) / a}) {
			if (t >= 0.0 && std::abs(_o.Z() + t * _d.Z()) <= _half_length)
				return t;
		}
		return kInf;
	}
}

void RayCaster::Update(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _exclude_model)
{
	shapes_.clear();
	ignored_ = 0;
	_ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
		[&](const gz::sim::Entity &_entity, const gz::sim::components::Collision *,
			const gz::sim::components::Geometry *_geometry) -> bool
		{
			const gz::sim::Entity model = gz::sim::topLevelModel(_entity, _ecm);
			if (model == _exclude_model)
				return true;
			const auto *is_static = _ecm.Component<gz::sim::components::Static>(model);

			Shape shape;
			shape.entity = _entity;
			shape.is_static = is_static && is_static->Data();
			shape.pose = gz::sim::worldPose(_entity, _ecm);
			const sdf::Geometry &g = _geometry->Data();
			switch (g.Type()) {
				case sdf::GeometryType::BOX:
					shape.type = Type::kBox;
					shape.size = g.BoxShape()->Size();
					break;
				case sdf::GeometryType::SPHERE:
					shape.type = Type::kSphere;
					shape.size.X(g.SphereShape()->Radius());
					break;
				case sdf::GeometryType::CYLINDER:
					shape.type = Type::kCylinder;
					shape.size.Set(g.CylinderShape()->Radius(), g.CylinderShape()->Length(), 0.0);
					break;
				case sdf::GeometryType::CAPSULE:
					shape.type = Type::kCapsule;
					shape.size.Set(g.CapsuleShape()->Radius(), g.CapsuleShape()->Length(), 0.0);
					break;
				case sdf::GeometryType::PLANE:
					shape.type = Type::kPlane;
					shape.size = g.PlaneShape()->Normal().Normalized();
					shape.plane_size = g.PlaneShape()->Size();
					break;
				default:
					ignored_++;
					return true;
			}
			shapes_.push_back(shape);
			return true;
		});
}

double RayCaster::Intersect(const Shape &_shape, const Vector3d &_o, const Vector3d &_d)
{
	switch (_shape.type) {
		case Type::kBox:
		{
			// Slab method in the box frame; 0 when the origin is inside
			double t_near = -kInf, t_far = kInf;
			for (int i = 0; i < 3; i++) {
				const double h = 0.5 * _shape.size[i];
				if (std::abs(_d[i]) < kEps) {
					if (std::abs(_o[i]) > h)
						return kInf;
					continue;
				}
				double t1 = (-h - _o[i]) / _d[i], t2 = (h - _o[i]) / _d[i];
				if (t1 > t2)
					std::swap(t1, t2);
				t_near = std::max(t_near, t1);
				t_far = std::min(t_far, t2);
				if (t_near > t_far)
					return kInf;
			}
			if (t_far < 0.0)
				return kInf;
			return std::max(0.0, t_near);
		}
		case Type::kSphere:
			return RaySphere(_o, _d, Vector3d::Zero, _shape.size.X());
		case Type::kCylinder:
		{
			const double r = _shape.size.X(), half = 0.5 * _shape.size.Y();
			if (_o.X() * _o.X() + _o.Y() * _o.Y() <= r * r && std::abs(_o.Z()) <= half)
				return 0.0;
			double t = RayCylinderSide(_o, _d, r, half);
			if (std::abs(_d.Z()) > kEps) {
				for (double z : {half, -half}) {
					const double tc = (z - _o.Z()) / _d.Z();
					const Vector3d p = _o + tc * _d;
					if (tc >= 0.0 && p.X() * p.X() + p.Y() * p.Y() <= r * r)
						t = std::min(t, tc);
				}
			}
			return t;
		}
		case Type::kCapsule:
		{
			const double r = _shape.size.X(), half = 0.5 * _shape.size.Y();
			return std::min({RayCylinderSide(_o, _d, r, half),
							 RaySphere(_o, _d, Vector3d(0, 0, half), r),
							 RaySphere(_o, _d, Vector3d(0, 0, -half), r)});
		}
		case Type::kPlane:
		{
			const Vector3d &n = _shape.size;
			const double dn = _d.Dot(n);
			if (std::abs(dn) < kEps)
				return kInf;
			const double t = -_o.Dot(n) / dn;
			if (t < 0.0)
				return kInf;
			// Finite plane: size along the plane axes (x, y rotated onto the normal)
			const Vector3d p = _o + t * _d;
			gz::math::Quaterniond rot;
			rot.SetFrom2Axes(Vector3d::UnitZ, n);
			if (std::abs(p.Dot(rot.RotateVector(Vector3d::UnitX))) > 0.5 * _shape.plane_size.X() ||
				std::abs(p.Dot(rot.RotateVector(Vector3d::UnitY))) > 0.5 * _shape.plane_size.Y())
				return kInf;
			return t;
		}
	}
	return kInf;
}

double RayCaster::Cast(const gz::sim::EntityComponentManager &_ecm,
					   const Vector3d &_origin, const Vector3d &_dir, double _max_range) const
{
	double nearest = kInf;
	for (const Shape &shape : shapes_) {
		const gz::math::Pose3d pose = shape.is_static ? shape.pose : gz::sim::worldPose(shape.entity, _ecm);
		// Ray in the frame of the shape
		const Vector3d o = pose.Rot().RotateVectorReverse(_origin - pose.Pos());
		const Vector3d d = pose.Rot().RotateVectorReverse(_dir);
		nearest = std::min(nearest, Intersect(shape, o, d));
	}
	return nearest <= _max_range ? nearest : kInf;
}
