// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/declarative/SensorModelParameters.h"
#include "../experiment_utils/declarative_environment.h"
#include "../visualization/declarative.h"

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
