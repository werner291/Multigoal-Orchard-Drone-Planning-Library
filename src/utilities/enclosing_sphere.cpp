//
// Created by werner on 6-3-23.
//

#include <geometry_msgs/geometry_msgs/msg/point.hpp>
#include <moveit_msgs/moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include "enclosing_sphere.h"

#include <CGAL/Cartesian_d.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Random.h>

namespace utilities {

	bodies::BoundingSphere
	compute_enclosing_sphere_around_points(const std::vector<Eigen::Vector3d> &points_eigen, const double padding) {

		const int N = 1000;                       // number of spheres
		const int D = 3;                          // dimension of points
		const int LOW = 0, HIGH = 10000;          // range of coordinates and radii
		//		typedef CGAL::Exact_rational              FT;
		typedef double                          FT;
		typedef CGAL::Cartesian_d<FT>             K;
		typedef CGAL::Min_sphere_of_spheres_d_traits_d<K,FT,D> Traits;
		typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
		typedef K::Point_d                        Point;
		typedef Traits::Sphere                    Sphere;
		std::vector<Sphere> S;                  // n spheres
		FT coord[D];                            // d coordinates
		CGAL::Random r;                         // random number generator

		for (const auto &point: points_eigen) {
			for (int j=0; j<D; ++j)
				coord[j] = point[j];
			Point p(D,coord,coord+D);             // random center...
			S.push_back(Sphere(p,0.0)); // ...and random radius
		}

		Min_sphere ms(S.begin(),S.end());       // check in the spheres
		assert(ms.is_valid());

		auto center_itr = ms.center_cartesian_begin();

		Eigen::Vector3d center;
		for (int i=0; i<D; ++i) {
			center[i] = *center_itr;
			++center_itr;
		}

		bodies::BoundingSphere bounding_sphere;
		bounding_sphere.center = center;
		bounding_sphere.radius = ms.radius();

		return bounding_sphere;

	}

	bodies::BoundingSphere
	compute_enclosing_sphere_around_leaves(const moveit_msgs::msg::PlanningScene &planning_scene_message,
										   const double padding) {

		std::vector<Eigen::Vector3d> points_eigen;

		for (const auto &col: planning_scene_message.world.collision_objects) {
			if (col.id == "leaves") {
				for (const auto &mesh: col.meshes) {
					for (auto v: mesh.vertices) {
						points_eigen.emplace_back(v.x, v.y, v.z);
					}
				}
			}
		}

		return compute_enclosing_sphere_around_points(points_eigen, padding);
	}


	std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const AppleTreePlanningScene &scene_info) {
		std::vector<geometry_msgs::msg::Point> mesh_points;
		for (const auto &col: scene_info.scene_msg->world.collision_objects) {
			if (col.id == "leaves") {

				// Extract the pose as an Isometry3d.
				Eigen::Isometry3d pose;
				pose.setIdentity();
				pose.translate(Eigen::Vector3d(col.pose.position.x, col.pose.position.y, col.pose.position.z));
				pose.rotate(Eigen::Quaterniond(col.pose.orientation.w, col.pose.orientation.x,
											  col.pose.orientation.y, col.pose.orientation.z));

				for (const auto &mesh: col.meshes) {
					for (auto v: mesh.vertices) {

						Eigen::Vector3d v_eigen(v.x, v.y, v.z);
						v_eigen = pose * v_eigen;

						v.x = v_eigen.x();
						v.y = v_eigen.y();
						v.z = v_eigen.z();

						mesh_points.push_back(v);
					}
				}
			}
		}
		return mesh_points;
	}

	std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const SimplifiedOrchard &orchard) {

		std::vector<geometry_msgs::msg::Point> leaf_vertices;

		for (const auto &[position, tree]: orchard.trees) {
			Eigen::Vector3d translation(position.x(), position.y(), 0.0);

			for (auto vertex: tree.leaves_mesh.vertices) {
				vertex.x += translation.x();
				vertex.y += translation.y();
				vertex.z += translation.z();

				leaf_vertices.push_back(vertex);
			}
		}

		return leaf_vertices;

	}
}