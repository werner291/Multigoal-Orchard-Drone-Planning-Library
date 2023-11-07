// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-11-23.
//

#include "cgal_chull_shortest_paths.h"

namespace mgodpl::cgal {

	void PathVisitor::operator()(Surface_mesh_shortest_path::halfedge_descriptor edge, Surface_mesh_shortest_path::FT t) {
		states.push_back(path_algo.face_location(edge, t));
		states.push_back(path_algo.face_location(mesh.opposite(edge), 1.0 - t));
	}

	void PathVisitor::operator()(Surface_mesh_shortest_path::vertex_descriptor vertex) {
		states.push_back(path_algo.face_location(vertex));
	}

	void PathVisitor::operator()(Surface_mesh_shortest_path::face_descriptor f,
								 Surface_mesh_shortest_path::Barycentric_coordinates location) {
		states.push_back({f, location});
	}
}