// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-3-23.
//

#ifndef NEW_PLANNERS_ORBITPATH_H
#define NEW_PLANNERS_ORBITPATH_H


#include <memory>
#include <Eigen/Core>
#include <utility>
#include <json/value.h>

class OrbitPath {

public:

	using UPtr = std::unique_ptr<OrbitPath>;

	[[nodiscard]] virtual Eigen::Vector3d at_t(double t) const = 0;

	virtual ~OrbitPath() = default;

};

class FlatOrbitPath : public OrbitPath {

public:
	Eigen::Vector3d center;
	double radius;
	double start_theta;
	double end_theta;

	FlatOrbitPath(Eigen::Vector3d center, double radius, double startTheta, double endTheta);

	[[nodiscard]] Eigen::Vector3d at_t(double t) const override;
};


class CylindricalOrbitPath : public OrbitPath {

public:
	Eigen::Vector3d center;
	double radius;
	double height;
	double start_theta;
	double end_theta;

	CylindricalOrbitPath(Eigen::Vector3d center, double radius, double height, double startTheta, double endTheta);

	[[nodiscard]] Eigen::Vector3d at_t(double t) const override;
};

class SphericalOscillatingOrbitPath : public OrbitPath {

public:

	Eigen::Vector3d center;
	double radius;
	double start_theta;
	double end_theta;

	SphericalOscillatingOrbitPath(Eigen::Vector3d center, double radius, double startTheta, double endTheta);

	[[nodiscard]] Eigen::Vector3d at_t(double t) const override;

};

class HelicalOrbit : public OrbitPath {

public:

	Eigen::Vector3d center;
	double radius;
	double height;
	double start_theta;
	double end_theta;

	HelicalOrbit(Eigen::Vector3d center, double radius, double height, double startTheta, double endTheta);

	[[nodiscard]] Eigen::Vector3d at_t(double t) const override;
};

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(FlatOrbitPath path);

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(CylindricalOrbitPath path);

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(SphericalOscillatingOrbitPath path);

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(HelicalOrbit orbit);

#endif //NEW_PLANNERS_ORBITPATH_H
