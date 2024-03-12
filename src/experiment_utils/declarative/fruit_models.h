// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#ifndef MGODPL_FRUIT_MODELS_H
#define MGODPL_FRUIT_MODELS_H

#include <variant>
#include <shape_msgs/msg/mesh.hpp>
#include "../../math/Vec3.h"

namespace mgodpl::declarative {

	/// Use the original set of fruit.
	struct Unchanged {
	};

	/// Select a random subset of the fruit.
	struct RandomSubset {
		int count; //< The number of elements to select from the set.
	};

	/// Replace the set of fruit with a new set.
	struct Replace {
		int count; //< The number of elements to replace in the set.
	};

	using FruitSubset = std::variant<Unchanged, RandomSubset, Replace>;

	/**
	 * A spherical fruit, defined by a center and a radius.
	 */
	struct SphericalFruit {
		math::Vec3d center; //< The center of the fruit.
		double radius{}; //< The radius of the fruit.
	};

	/**
	 * A mesh fruit, defined by a mesh and a center.
	 */
	struct MeshFruit {
		shape_msgs::msg::Mesh mesh; //< The mesh of the fruit.
		math::Vec3d center; //< The center of the fruit.
	};

}

#endif //MGODPL_FRUIT_MODELS_H
