// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <gtest/gtest.h>
#include <geometric_shapes/shapes.h>
#include <Eigen/Core>
#include <ompl/util/RandomNumbers.h>
#include "../src/exploration/SurfletVolume.h"
#include "../src/utilities/shape_generation.h"

TEST(SurfletVolume, single_sphere) {

	double radius = 1.0;
	Eigen::Vector3d center(0.5, 0, 0);

	SurfletVolume volume1;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius + center;
		s.normal = pt;
		volume1.add(s);
	}

	double margin = 0.1;

	// Now, generate 100 random points outside the volume, and 100 random points inside the volume, by at least the margin,
	// and check that the volume is correctly identified as inside or outside.

	for (int i = 0; i < 500; ++i) {

		Eigen::Vector3d p = Eigen::Vector3d::Random() * 2.0 * radius + center;

		std::cout << p.transpose() << std::endl;

		if ((p - center).norm() > radius + margin) {
			ASSERT_FALSE(volume1.isInside(p));
		} else if ((p - center).norm() < radius - margin) {
			ASSERT_TRUE(volume1.isInside(p));
		}

	}


}


TEST(SurfletVolume, union_of_two_spheres) {

	double radius1 = 0.9;
	Eigen::Vector3d center1(0.5, 0, 0);

	double radius2 = 1.3;
	Eigen::Vector3d center2(-0.5, 0, 0.1);

	SurfletVolume volume1;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius1 + center1;
		s.normal = pt;
		volume1.add(s);
	}

	SurfletVolume volume2;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius2 + center2;
		s.normal = pt;
		volume2.add(s);
	}

	SurfletVolume volume = volume1.unionWith(volume2);

	double margin = 0.1;

	// Now, generate 100 random points outside the volume, and 100 random points inside the volume, by at least the margin,
	// and check that the volume is correctly identified as inside or outside.

	for (int i = 0; i < 500; ++i) {

		Eigen::Vector3d p = Eigen::Vector3d::Random() * 2.0 * radius1 + center1;

		std::cout << p.transpose() << std::endl;

		if ((p - center1).norm() > radius1 + margin && (p - center2).norm() > radius2 + margin) {
			ASSERT_FALSE(volume.isInside(p));
		} else if ((p - center1).norm() < radius1 - margin || (p - center2).norm() < radius2 - margin) {
			ASSERT_TRUE(volume.isInside(p));
		}

	}

}

TEST(SurfletVolume, disjoint_union_sum) {

	double radius1 = 0.9;
	Eigen::Vector3d center1(2.0, 0, 0);

	double radius2 = 1.3;
	Eigen::Vector3d center2(-2.0, 0, 0.1);

	SurfletVolume volume1;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius1 + center1;
		s.normal = pt;
		volume1.add(s);
	}

	SurfletVolume volume2;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius2 + center2;
		s.normal = pt;
		volume2.add(s);
	}

	SurfletVolume volume = volume1.unionWith(volume2);

	EXPECT_EQ(volume1.getSurflets().size() + volume2.getSurflets().size(), volume.getSurflets().size());

}


TEST(SurfletVolume, self_union_identity) {

	double radius1 = 1.0;
	Eigen::Vector3d center1(0.0, 0, 0);

	SurfletVolume volume1;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius1 + center1;
		s.normal = pt;
		volume1.add(s);
	}

	SurfletVolume volume = volume1.unionWith(volume1);

	EXPECT_EQ(volume.getSurflets().size(), volume1.getSurflets().size());

}

TEST(SurfletVolume, conjoined_union_sum) {

	double radius1 = 1.0;
	Eigen::Vector3d center1(0.1, 0, 0);

	double radius2 = 1.0;
	Eigen::Vector3d center2(-0.1, 0, 0.0);

	SurfletVolume volume1;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius1 + center1;
		s.normal = pt;
		volume1.add(s);
	}

	SurfletVolume volume2;

	for (const auto &pt: spherifiedCubeVertices(3)) {
		SurfletVolume::Surflet s;
		s.point = pt * radius2 + center2;
		s.normal = pt;
		volume2.add(s);
	}

	SurfletVolume volume = volume1.unionWith(volume2);

	EXPECT_GE(volume.getSurflets().size(), volume1.getSurflets().size());
	EXPECT_GE(volume.getSurflets().size(), volume2.getSurflets().size());
	EXPECT_LT(volume.getSurflets().size(), volume1.getSurflets().size() + volume2.getSurflets().size());

}

TEST(SurfletVolume, swept_volume_angle) {

	Eigen::Vector3d center1(0, 0, 0);
	Eigen::Vector3d center2(0, 0, 2);
	Eigen::Vector3d center3(0, 2, 2);

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segments{{center1, center2},
																	  {center2, center3}};

	SurfletVolume swept_volume;

	const size_t num_points = 100;

	for (const auto &[p1, p2]: segments) {

		for (size_t i = 0; i <= num_points; ++i) {

			SurfletVolume sphere;

			double t = (double) i / (double) num_points;

			Eigen::Vector3d center = p1 + (p2 - p1) * t;

			for (const auto &pt: spherifiedCubeVertices(3)) {
				SurfletVolume::Surflet s;
				s.point = pt * 1.0 + center;
				s.normal = pt;
				sphere.add(s);
			}

			swept_volume = swept_volume.unionWith(sphere);

			std::cout << "N Surflets: " << swept_volume.getSurflets().size() << std::endl;

		}

	}

	ompl::RNG rng;

	// Now, we sample points within the theoretical swept volume, and check that they are all inside the swept volume.

	for (int i = 0; i < 1000; ++i) {

		// Pick a point at random.

		Eigen::Vector3d p(rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 6.0), rng.uniformReal(-2.0, 6.0));


		// Find the closest point on one of the line segments.

		Eigen::Vector3d closest_point;
		double closest_distance = std::numeric_limits<double>::infinity();

		for (const auto &[p1, p2]: segments) {

			double projection_t = (p - p1).dot(p2 - p1) / (p2 - p1).squaredNorm();

			projection_t = std::max(0.0, std::min(1.0, projection_t));

			Eigen::Vector3d projection = p1 + (p2 - p1) * projection_t;

			double distance = (p - projection).norm();

			if (distance < closest_distance) {
				closest_distance = distance;
				closest_point = projection;
			}

		}

		double distance = (p - closest_point).norm();

		if (distance < 0.9) {
			EXPECT_TRUE(swept_volume.isInside(p));
			std::cout << "Point " << p.transpose() << " is inside." << std::endl;
		} else if (distance > 1.1) {
			EXPECT_FALSE(swept_volume.isInside(p));
		}

	}


}