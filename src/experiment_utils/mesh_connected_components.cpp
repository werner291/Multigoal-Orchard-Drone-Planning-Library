// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#include "mesh_connected_components.h"

#include <unordered_map>
#include <cassert>

std::vector<size_t> index_vector(const size_t& n) {
	std::vector<size_t> result(n);
	for (size_t i = 0; i < n; i++) {
		result[i] = i;
	}
	return result;
}

namespace mgodpl {

	std::vector<std::vector<size_t>> connected_vertex_components(const Mesh &mesh) {

		// Create a numeric index of the list of vertices.
		/// Initially, just the range [0,num_vertices), but will be updated to point to the connected component ID.
		auto connected_component_ids = index_vector(mesh.vertices.size());

		// Map of connected components, with identifiers mapping to vector of vertex indices.
		std::unordered_map<size_t, std::vector<size_t>> connected_components;

		// Starts out as singletons of each vertex.
		for (const auto &vertex_id: connected_component_ids) {
			connected_components.insert({vertex_id, {vertex_id}});
		}

		// Iterate over all mesh triangles.
		for (const auto &triangle: mesh.triangles) {
			// And over every vertex-to-vertex connection in the triangle.
			// We only need to consider two, since connections are transitive.
			for (size_t i: {0, 1}) {

				// Look up the connected component IDs for each.
				size_t ccidA = connected_component_ids[triangle[i]];
				size_t ccidB = connected_component_ids[triangle[i + 1]];

				// If they're in different components, merge the components.
				if (ccidA != ccidB) {

					// Add all the vertices from the second component to the first.
					for (const auto &in_ccb: connected_components[ccidB]) {
						connected_components[ccidA].push_back(in_ccb);
						// Update the backpointer to the component ID.
						connected_component_ids[in_ccb] = ccidA;
					}

					// Remove the second component.
					connected_components.erase(ccidB);
				}

			}
		}

		// Return a vector of connected components.
		std::vector<std::vector<size_t>> result;
		for (auto [_id, contents]: connected_components) {
			result.push_back(std::move(contents));
		}
		return result;
	}


	std::vector<Mesh> break_down_to_connected_components(const Mesh &combined_mesh) {

		// Extract the connected components.
		auto cc = connected_vertex_components(combined_mesh);

		// Allocate n meshes for the n connected components.
		std::vector<Mesh> mesh_components(cc.size());

		// Split up the vertices and build up an index translation map.
		std::vector<std::pair<size_t, size_t>> index_map(combined_mesh.vertices.size());

		// For every connected component...
		for (size_t i = 0; i < cc.size(); i++) {
			// ...and for every vertex in the connected component...
			for (size_t j = 0; j < cc[i].size(); j++) {
				// ...record the connected component ID and new vertex ID within the component in the index map,
				index_map[cc[i][j]] = std::make_pair(i, j);
				// ...and add the vertex to the mesh for the connected component.
				mesh_components[i].vertices.push_back(combined_mesh.vertices[cc[i][j]]);
			}
		}

		// For every triangle in the combined mesh...
		for (auto triangle: combined_mesh.triangles) {

			// Look up the connected component ID and new vertex ID of the triangle's vertices in the index map.
			auto &v0 = index_map[triangle[0]];
			auto &v1 = index_map[triangle[1]];
			auto &v2 = index_map[triangle[2]];

			// Sanity check: the three vertices should all be in the same connected component.
			assert(v0.first == v1.first && v1.first == v2.first);

			// Add the triangle to the mesh for the connected component.
			mesh_components[v0.first].triangles.push_back(triangle);
			mesh_components[v0.first].triangles.back()[0] = v0.second;
			mesh_components[v0.first].triangles.back()[1] = v1.second;
			mesh_components[v0.first].triangles.back()[2] = v2.second;
		}

		return mesh_components;
	}
}
