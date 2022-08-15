#include <Eigen/Geometry>
#include <ompl/util/RandomNumbers.h>
#include <shape_msgs/msg/mesh.h>
#include <bullet/HACD/hacdHACD.h>
#include <boost/range/irange.hpp>
#include "general_utilities.h"

/**
 * Given two vector4's, produce a third vector perpendicular to the inputs.
 * The result lies in the w=0 plane (Eigen::Vector4 is (x,y,z,w)).
 */
Eigen::Vector4d any_perpendicular_of_two(const Eigen::Vector4d &a, const Eigen::Vector4d &b) {

	// This is basically just the 3D vector cross product with a 0 tacked on the end.
	// w-component of the original vectors is ignored.
	return Eigen::Vector4d(a.y() * b.z() - b.y() * a.z(),
						   b.x() * a.z() - a.x() * b.z(),
						   a.x() * b.y() - b.x() * a.y(),
						   0);
}

/**
 * Perform the generalized cross product where a 4-vector is produced which lies perpendicular
 * to the three input vectors, assuming all are nonzero and linearly independent.
 */
Eigen::Vector4d cross_three(const Eigen::Vector4d &u, const Eigen::Vector4d &v, const Eigen::Vector4d &t) {
	// Source: https://www.researchgate.net/publication/318543243_Vector_Cross_Product_in_4D_Euclidean_Space_A_Maple_worksheet
	return Eigen::Vector4d((u.w() * v.z() - u.z() * v.w()) * t.y() + (-u.w() * v.y() + u.y() * v.w()) * t.z() +
						   (-u.y() * v.z() + u.z() * v.y()) * t.w(),
						   (-u.w() * v.z() + u.z() * v.w()) * t.x() + (u.w() * v.x() - u.x() * v.w()) * t.z() +
						   (u.x() * v.z() - u.z() * v.x()) * t.w(),
						   (u.w() * v.y() - u.y() * v.w()) * t.x() + (-u.w() * v.x() + u.x() * v.w()) * t.y() +
						   (-u.x() * v.y() + u.y() * v.x()) * t.w(),
						   (u.y() * v.z() - u.z() * v.y()) * t.x() + (-u.x() * v.z() + u.z() * v.x()) * t.y() +
						   (u.x() * v.y() - u.y() * v.x()) * t.z());
}

// This could be a publication on its' own. Let's keep this around.
[[maybe_unused]] Eigen::Quaterniond
sampleInformedQuaternion(const Eigen::Quaterniond &qa, const Eigen::Quaterniond &qb, const double max_distance) {

	// Unfortunately, Eigen really only seems to consider Quaternions as useful
	// for rotations, so we convert to 4D vectors to get a richer API.
	Eigen::Vector4d ra(qa.x(), qa.y(), qa.z(), qa.w()), rb(qb.x(), qb.y(), qb.z(), qb.w());

	// Angle between quaternions (in OMPL and MoveIt) is defined using
	// the absolute value of the dot product. Hence, if the input quaternions
	// are more than 90 degrees apart, we can reflect one to the other side
	// of the hypersphere and simply do the calculations as if the absolute
	// value wasn't there.
	if (ra.dot(rb) < 0.0) {
		rb *= -1.0;
	}

	// Quick sanity check: rotation quaternions should be unit-norm.
	// Note: we interpret these as points on a 4D hypersphere.
	assert(abs(ra.norm() - 1.0) < 1.0e-10);
	assert(abs(rb.norm() - 1.0) < 1.0e-10);

	// Get an RNG for sampling.
	ompl::RNG rng;

	// The distance between the two input rotations, defined as the the arc cosine of the dot product.
	double between_inputs = std::acos(ra.dot(rb));

	// Input validation: the triangle inequality requires that any resulting distance
	// is at least the geodesic distance between the two inputs, minus some absolute-value
	// adjustments.
	assert(max_distance >= between_inputs);

	// Output vector.
	Eigen::Vector4d result_vec;

	if (max_distance + between_inputs >= 2.0 * M_PI) {
		// Special case: the tolerance is so large that any orientation will fulfill the requirement,
		// even if the resulting movement means wrapping all the way around the back of the sphere.
		result_vec = Eigen::Vector4d(rng.gaussian01(), rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
		result_vec.normalize();

	} else {

		/*
		 * The way the method works is as follows: we consider a stereographic projection from the (0,0,0,-1)
		 * point of the unit hypersphere.
		 *
		 * Then, we pick a sample from a carefully-chosen ellipsoid in the resulting subspace, such that
		 * when inverse-projecting that ellipsoid onto the unit sphere, it exactly covers the area from
		 * which samples can be drawn that satisfy the informed sampling condition.
		 *
		 * See https://www.geogebra.org/calculator/sbt2csng for a 3D demonstration of this calculation;
		 * the math trivially scales to 4D.
		 *
		 * For the relationship between spherical ellipses and regular ellipses, see this thesis:
		 * https://www.geometrie.tuwien.ac.at/theses/pdf/diplomarbeit_tranacher.pdf
		 */

		// Compute the semi-major and semi-minor axes of this ellipse.
		// Note: the ellipse is stretched only in one dimension.
		double sma = std::tan(max_distance / 4.0);
		double smi = std::tan(std::acos(std::cos(max_distance / 2.0) / std::cos(between_inputs / 2.0)) / 2.0);

		// Pick a sample from a unit sphere, the multiply such that it effectively is drawn from an ellipsoid
		// stretched to fulfill the aforementioned properties.
		std::vector<double> sample_xy(3);
		rng.uniformInBall(1.0, sample_xy);
		Eigen::Vector3d sample(sample_xy[0] * sma, sample_xy[1] * smi, sample_xy[2] * smi);

		// Apply an inverse stereographic projection to map the point from the ellipsoid to the unit hypersphere.
		// The projection point is (0,0,0,-1).
		// TODO: compensate for the distortion that this probably inflicts on the distribution of samples.
		Eigen::Vector4d on_sphere(
				2.0 * sample.x() / (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z()),
				2.0 * sample.y() / (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z()),
				2.0 * sample.z() / (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z()),
				(1.0 - sample.x() * sample.x() - sample.y() * sample.y() - sample.z() * sample.z()) /
				(1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z()));

		// Absolute value in the angle calculations makes it so that there is actually
		// an area on the exact opposite of the unit hypersphere that is also a valid
		// sampling region of the same size. We pick from there with 50% probability.
		if (rng.uniformBool()) {
			sample.z() *= -1.0;
		}

		// Sanity check: did the inverse-projection actually project he sample into the sphere?
		assert(abs(on_sphere.norm() - 1.0) < 1.0e-10);

		// Now, we need to "rotate" around the hypersphere such that the projection
		// of the ellipsoid is actually centered on the input quaternions.

		// A vector pointing in the direction from one point to the other.
		// the ellipsoid is stretched along this direction.
		auto fw = (ra - rb).normalized();

		// A normal vector of the hypersphere at a point halfway between the two quaternions.
		auto up = (0.5 * ra + 0.5 * rb).normalized();
		// Then, we obtain two more perpendicular 4-vectors to build a local coordinate frame.
		Eigen::Vector4d p1 = any_perpendicular_of_two(up, fw).normalized();
		Eigen::Vector4d p2 = cross_three(up, fw, p1);

		// Assemble the vectors into a matrix
		Eigen::Matrix4d xf;
		xf << fw.x(), p1.x(), p2.x(), up.x(), fw.y(), p1.y(), p2.y(), up.y(), fw.z(), p1.z(), p2.z(), up.z(), fw.w(), p1
				.w(), p2.w(), up.w();

		// Use the matrix to transform our sample into the coordinate frame centered onto the two quaternions.
		result_vec = xf * on_sphere;
	}

	// Convert the result 1-to-1 to a quaternion, and return it.
	return Eigen::Quaterniond(result_vec.w(), result_vec.x(), result_vec.y(), result_vec.z());
}

std::vector<std::vector<size_t>> connected_vertex_components(const shape_msgs::msg::Mesh &mesh) {

	// Create a numeric index of the list of vertices.
	/// Initially, just the range [0,num_vertices), but will be updated to point to the connected component ID.
	auto connected_component_ids = index_vector(mesh.vertices);

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
			size_t ccidA = connected_component_ids[triangle.vertex_indices[i]];
			size_t ccidB = connected_component_ids[triangle.vertex_indices[i + 1]];

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

std::vector<shape_msgs::msg::Mesh> convex_decomposition(const shape_msgs::msg::Mesh &mesh, const double concavity) {

	// Initialize the HACD library
	HACD::HACD hacd;

	// Convert the Eigen vertices to HACD points.
	auto points = boost::copy_range<std::vector<HACD::Vec3<double>>>(
			mesh.vertices | boost::adaptors::transformed([](const auto &pt) {
				return HACD::Vec3(pt.x, pt.y, pt.z);
			}));

	// Convert the Eigen triangles to HACD faces.
	auto triangles = boost::copy_range<std::vector<HACD::Vec3<long>>>(
			mesh.triangles | boost::adaptors::transformed([](const auto &tr) {
				return HACD::Vec3((long) tr.vertex_indices[0],
								  (long) tr.vertex_indices[1],
								  (long) tr.vertex_indices[2]);
			}));

	// Send the triangles and points to the HACD library.
	hacd.SetPoints(points.data());
	hacd.SetNPoints(points.size());

	hacd.SetTriangles(triangles.data());
	hacd.SetNTriangles(triangles.size());

	// Set the concavity (parameter to affect solution size vs quality and expense)
	hacd.SetConcavity(concavity);

	// Run the HACD algorithm.
	hacd.Compute();

	// Extract the meshes from HACD.
	// We iterate over the cluster IDs (that's what HACD calls the connected components)
	return boost::copy_range<std::vector<shape_msgs::msg::Mesh>>(
			boost::irange<size_t>(0, hacd.GetNClusters()) | boost::adaptors::transformed([&](size_t ch_i) {

				// Empty mesh.
				shape_msgs::msg::Mesh sub_mesh;

				// Extract the points and triangles from HACD.
				std::vector<HACD::Vec3<double>> ch_points(hacd.GetNPointsCH(ch_i));
				std::vector<HACD::Vec3<long>> ch_triangles(hacd.GetNTrianglesCH(ch_i));
				hacd.GetCH(ch_i, ch_points.data(), ch_triangles.data());

				// Convert the HACD points to ROS points.
				sub_mesh.vertices = boost::copy_range<std::vector<geometry_msgs::msg::Point>>(
						ch_points | boost::adaptors::transformed([](const auto &pt) {
							geometry_msgs::msg::Point ros_pt;
							ros_pt.x = pt.X();
							ros_pt.y = pt.Y();
							ros_pt.z = pt.Z();
							return ros_pt;
						}));

				// Convert the HACD triangles to ROS triangles.
				sub_mesh.triangles = boost::copy_range<std::vector<shape_msgs::msg::MeshTriangle>>(
						ch_triangles | boost::adaptors::transformed([](const auto &pt) {
							shape_msgs::msg::MeshTriangle tri;
							tri.vertex_indices[0] = pt.X();
							tri.vertex_indices[1] = pt.Y();
							tri.vertex_indices[2] = pt.Z();
							return tri;
						}));

				// Sanity check: make sure all vertex indices are in bounds.
				for (const auto &tri: sub_mesh.triangles) {
					assert(tri.vertex_indices[0] < sub_mesh.vertices.size());
					assert(tri.vertex_indices[1] < sub_mesh.vertices.size());
					assert(tri.vertex_indices[2] < sub_mesh.vertices.size());
				}

				return sub_mesh;
			}));

}

void checkPtc(const ompl::base::PlannerTerminationCondition &ptc) {
	if (ptc()) { // If the termination condition is met, throw an exception.
		throw PlanningTimeout();
	}
}

void fixWinding(shape_msgs::msg::Mesh &mesh) {

	Eigen::Vector3d centroid(0, 0, 0);
	for (const auto& v : mesh.vertices) {
		centroid += Eigen::Vector3d(v.x, v.y, v.z);
	}
	centroid /= (double) mesh.vertices.size();


	for (auto& t : mesh.triangles) {
		Eigen::Vector3d v0(mesh.vertices[t.vertex_indices[0]].x, mesh.vertices[t.vertex_indices[0]].y, mesh.vertices[t.vertex_indices[0]].z);
		Eigen::Vector3d v1(mesh.vertices[t.vertex_indices[1]].x, mesh.vertices[t.vertex_indices[1]].y, mesh.vertices[t.vertex_indices[1]].z);
		Eigen::Vector3d v2(mesh.vertices[t.vertex_indices[2]].x, mesh.vertices[t.vertex_indices[2]].y, mesh.vertices[t.vertex_indices[2]].z);
		Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0);
		if (normal.dot(centroid - v0) < 0) {
			std::swap(t.vertex_indices[1], t.vertex_indices[2]);
		}
	}

}

const char *PlanningTimeout::what() const noexcept {
	return "Planning timeout";
}
