// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "OrbitPath.h"

#include <utility>
#include "utilities/json_utils.h"

FlatOrbitPath::FlatOrbitPath(Eigen::Vector3d center, double radius, double startTheta, double endTheta)
		: center(std::move(center)), radius(radius), start_theta(startTheta), end_theta(endTheta) {
}

Eigen::Vector3d FlatOrbitPath::at_t(double t) const {

	double theta = start_theta + t * (end_theta - start_theta);

	Eigen::Vector3d ray_unit(std::cos(theta), std::sin(theta), 0);

	return center + radius * ray_unit;

}

CylindricalOrbitPath::CylindricalOrbitPath(Eigen::Vector3d center,
										   double radius,
										   double height,
										   double startTheta,
										   double endTheta)
		: center(std::move(center)), radius(radius), height(height), start_theta(startTheta), end_theta(endTheta) {
}

Eigen::Vector3d CylindricalOrbitPath::at_t(double t) const {

	double theta = start_theta + t * (end_theta - start_theta);

	Eigen::Vector3d ray_unit(std::cos(theta), std::sin(theta), 0);

	double vertical = std::sin(t * 2.0 * M_PI) * height;

	return center + radius * ray_unit + Eigen::Vector3d(0, 0, vertical);

}

SphericalOscillatingOrbitPath::SphericalOscillatingOrbitPath(Eigen::Vector3d center,
															 double radius,
															 double startTheta,
															 double endTheta)
		: center(std::move(center)), radius(radius), start_theta(startTheta), end_theta(endTheta) {
}

Eigen::Vector3d SphericalOscillatingOrbitPath::at_t(double t) const {

	double theta = start_theta + t * (end_theta - start_theta);

	double vertical = std::sin(t * 2.0 * M_PI) * radius;

	Eigen::Vector3d ray_unit = Eigen::Vector3d(std::cos(theta), std::sin(theta), vertical).normalized();

	return center + radius * ray_unit;

}

HelicalOrbit::HelicalOrbit(Eigen::Vector3d center, double radius, double height, double startTheta, double endTheta)
		: center(std::move(center)), radius(radius), height(height), start_theta(startTheta), end_theta(endTheta) {
}

Eigen::Vector3d HelicalOrbit::at_t(double t) const {

	double theta = start_theta + t * (end_theta - start_theta);

	Eigen::Vector3d ray_unit(std::cos(theta), std::sin(theta), 0);

	double vertical = (t - 0.5) * height * 2.0;

	return center + radius * ray_unit + Eigen::Vector3d(0, 0, vertical);

}

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(FlatOrbitPath path) {

	Json::Value json;
	json["type"] = "flat";
	json["center"] = toJSON(path.center);
	json["radius"] = path.radius;
	json["start_theta"] = path.start_theta;
	json["end_theta"] = path.end_theta;

	OrbitPath::UPtr ptr = std::make_unique<FlatOrbitPath>(path);

	return std::make_pair(json, std::make_unique<FlatOrbitPath>(path));

}

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(CylindricalOrbitPath path) {

	Json::Value json;
	json["type"] = "cylindrical";
	json["center"] = toJSON(path.center);
	json["radius"] = path.radius;
	json["height"] = path.height;
	json["start_theta"] = path.start_theta;
	json["end_theta"] = path.end_theta;

	auto ptr = std::make_unique<CylindricalOrbitPath>(path);

	return std::make_pair(json, std::move(ptr));

}

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(SphericalOscillatingOrbitPath path) {

	Json::Value json;
	json["type"] = "spherical_oscillating";
	json["center"] = toJSON(path.center);
	json["radius"] = path.radius;
	json["start_theta"] = path.start_theta;
	json["end_theta"] = path.end_theta;

	auto ptr = std::make_unique<SphericalOscillatingOrbitPath>(path);

	return std::make_pair(json, std::move(ptr));
}

std::pair<Json::Value, OrbitPath::UPtr> pairWithJson(HelicalOrbit orbit) {

	Json::Value json;
	json["type"] = "helical";
	json["center"] = toJSON(orbit.center);
	json["radius"] = orbit.radius;
	json["height"] = orbit.height;
	json["start_theta"] = orbit.start_theta;
	json["end_theta"] = orbit.end_theta;

	auto ptr = std::make_unique<HelicalOrbit>(orbit);
	return std::make_pair(json, std::move(ptr));

}
