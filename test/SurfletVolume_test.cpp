// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <gtest/gtest.h>
#include <geometric_shapes/shapes.h>
#include <Eigen/Core>
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