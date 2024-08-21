// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/math/bv/OBB.h>
#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/declarative/SensorModelParameters.h"
#include "../experiment_utils/declarative_environment.h"
#include "../visualization/declarative.h"
#include "../visualization/ladder_trace.h"
#include "../experiment_utils/default_colors.h"
#include "../planning/RobotModel.h"
#include "../planning/probing_motions.h"
#include "../planning/state_tools.h"

using namespace mgodpl;
using namespace declarative;
using namespace experiments;
using namespace visualization;

REGISTER_VISUALIZATION(declarative_visualization)
{

	TreeModelParameters params {
			.name = "appletree",
			.leaf_scale = 1.5,
			.fruit_subset = Replace { 100 },
			.seed = 42
	};

	mgodpl::experiments::TreeModelCache cache;
	random_numbers::RandomNumberGenerator rng(42);

	const auto& model = instantiate_tree_model(params, cache, rng);

	visualize(viewer, model);

	const auto& scan_points = generate_scannable_points(model.fruit_models, 100, rng);

	visualize(viewer, scan_points);
	visualize(viewer, instantiatePath({CircularOrbitParameters{.radius=1.0}},*model.tree_model));
	visualize(viewer, instantiatePath({CircularOrbitParameters{.radius=2.0}},*model.tree_model));
	visualize(viewer, instantiatePath({CircularOrbitParameters{.radius=3.0}},*model.tree_model));
	visualize(viewer, instantiatePath({CircularOrbitParameters{.radius=4.0}},*model.tree_model));

	viewer.start();

}

REGISTER_VISUALIZATION(declarative_fullpath) {

	mgodpl::declarative::PointScanEvalParameters eval_params {
			.tree_params = TreeModelParameters {
					.name = "appletree",
					.leaf_scale = 1.5,
					.fruit_subset = Unchanged {}, //Replace {100},
					.seed = 42
			},
			.sensor_params = SensorScalarParameters {
					.maxViewDistance = INFINITY,
					.minViewDistance = 0.0,
					.fieldOfViewAngle = M_PI / 3.0,
					.maxScanAngle = M_PI / 3.0,
			}
	};

	mgodpl::experiments::TreeModelCache environment_cache;

	random_numbers::RandomNumberGenerator rng(42);

	const PointScanEnvironment &env = create_environment(eval_params, environment_cache);

	ShellPathPlanningMethod method;

	RobotPath path = method.plan_static(
			env.robot,
			env.tree_model->meshes.trunk_mesh,
			env.scaled_leaves,
			fruit_positions_from_models(env.fruit_models),
			env.initial_state
			);

	visualize_ladder_trace(env.robot, path, viewer);

	viewer.addMesh(env.tree_model->meshes.trunk_mesh, WOOD_COLOR);
	viewer.addMesh(env.scaled_leaves, LEAF_COLOR);

	if (const auto& mesh = std::get_if<std::vector<MeshFruit>>(&env.fruit_models)) {
		for (const auto &fruit_mesh: *mesh) {
			viewer.addMesh(fruit_mesh.mesh, FRUIT_COLOR);
		}
	} else if (const auto& sphere = std::get_if<std::vector<SphericalFruit>>(&env.fruit_models)) {
		for (const auto &fruit_center: *sphere) {
			viewer.addSphere(fruit_center.radius, fruit_center.center, FRUIT_COLOR);
		}
	}

	viewer.start();
}
