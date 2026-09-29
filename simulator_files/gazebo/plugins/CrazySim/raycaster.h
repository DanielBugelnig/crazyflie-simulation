/*
 * Ray casting against the collision shapes of the world, without rendering.
 *
 * gz-sim 7 has no ray query for systems and a gpu_lidar costs a render pass
 * per update, which slows the simulation below real time (the SITL firmware
 * runs on wall clock time). Boxes, spheres, cylinders, capsules and planes
 * are supported; meshes are ignored.
 */
#ifndef RAYCASTER_H_
#define RAYCASTER_H_

#include <limits>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace crazyflie_interface
{
	class RayCaster
	{
		// Collects the collision shapes of all models except _exclude_model
		public: void Update(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _exclude_model);

		// Distance along the unit direction _dir to the first surface, +inf without a hit
		public: double Cast(const gz::sim::EntityComponentManager &_ecm,
							const gz::math::Vector3d &_origin, const gz::math::Vector3d &_dir,
							double _max_range) const;

		public: size_t ShapeCount() const { return shapes_.size(); }
		public: size_t IgnoredCount() const { return ignored_; }

		private: enum class Type { kBox, kSphere, kCylinder, kCapsule, kPlane };
		private: struct Shape
		{
			Type type;
			gz::sim::Entity entity;		// collision, for the pose of moving models
			bool is_static;
			gz::math::Pose3d pose;		// world pose (static models)
			gz::math::Vector3d size;	// box size | radius, length | plane normal
			gz::math::Vector2d plane_size;
		};

		private: static double Intersect(const Shape &_shape, const gz::math::Vector3d &_o,
										 const gz::math::Vector3d &_d);

		private: std::vector<Shape> shapes_;
		private: size_t ignored_ = 0;
	};
}

#endif
