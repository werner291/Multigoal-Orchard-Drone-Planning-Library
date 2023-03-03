#include "math_utils.h"
#include <Eigen/Geometry>
#include <iostream>
#include <random>


std::pair<double, double>
closest_point_on_line(const Eigen::ParametrizedLine<double, 3> &l1,
					  const Eigen::ParametrizedLine<double, 3> &l2) {

	// Direction vectors of both lines.
	const Eigen::Vector3d& d1 = l1.direction();
	const Eigen::Vector3d& d2 = l2.direction();

	// Formula from	https://en.wikipedia.org/wiki/Skew_lines#Nearest_points

	// The edge connecting the two points of on_which_mesh approach must be perpendicular to both vectors.
	// Hence, we can use the cross product to find a direction vector of the edge. (but not the magnitude)
	Eigen::Vector3d n = d1.cross(d2);

	// Compute the normal of the plane containing d1 perpendicular to n.
	Eigen::Vector3d n1 = d1.cross(n);

	// Compute the normal of the plane containing d2 perpendicular to n.
	Eigen::Vector3d n2 = d2.cross(n);

	return {n2.dot(l2.origin() - l1.origin()) / n2.dot(d1), n1.dot(l1.origin() - l2.origin()) / n1.dot(d2)};

}

double projectionParameter(const Eigen::ParametrizedLine<double, 3> &line, const Eigen::Vector3d &point) {
	return (point - line.origin()).dot(line.direction()) / line.direction().squaredNorm();
}

Eigen::Vector3d project_barycentric(const Eigen::Vector3d &qp,
									const Eigen::Vector3d &va,
									const Eigen::Vector3d &vb,
									const Eigen::Vector3d &vc) {
	// u=P2−P1
	Eigen::Vector3d u = vb - va;
	// v=P3−P1
	Eigen::Vector3d v = vc - va;
	// n=u×v
	Eigen::Vector3d n = u.cross(v);
	// w=P−P1
	Eigen::Vector3d w = qp - va;
	// Barycentric coordinates of the projection P′of P onto T:
	// γ=[(u×w)⋅n]/n²
	double gamma = u.cross(w).dot(n) / n.dot(n);
	// β=[(w×v)⋅n]/n²
	double beta = w.cross(v).dot(n) / n.dot(n);

	// Must sum to 1.
	double alpha = 1 - gamma - beta;

	return {alpha, beta, gamma};
}

Eigen::Vector3d closest_point_on_triangle(const Eigen::Vector3d &p,
										  const Eigen::Vector3d &va,
										  const Eigen::Vector3d &vb,
										  const Eigen::Vector3d &vc) {

	Eigen::Vector3d barycentric = project_barycentric(p, va, vb, vc);

	if (0 <= barycentric[0] && barycentric[0] <= 1 && 0 <= barycentric[1] && barycentric[1] <= 1 && 0 <= barycentric[2] && barycentric[2] <= 1) {
		return va * barycentric[0] + vb * barycentric[1] + vc * barycentric[2];
	} else {

		Eigen::Vector3d pt_ab = math_utils::Segment3d(va, vb).closest_point(p);
		Eigen::Vector3d pt_bc = math_utils::Segment3d(vb, vc).closest_point(p);
		Eigen::Vector3d pt_ca = math_utils::Segment3d(vc, va).closest_point(p);

		return math_utils::closest_point_in_list({pt_ab, pt_bc, pt_ca}, p);

	}

}

Eigen::Vector3d closest_point_on_open_triangle(const Eigen::Vector3d &p, const OpenTriangle &triangle) {

	// Same as closest_point_on_triangle, but we don't limit one side of the triangle.

	Eigen::Vector3d barycentric = project_barycentric(p,
													  triangle.apex,
													  triangle.apex + triangle.dir1,
													  triangle.apex + triangle.dir2);

	assert(abs(barycentric[0] + barycentric[1] + barycentric[2] - 1) < 1e-6);

	// Note: 0 <= barycentric[0] case deliberately omitted.
	if (barycentric[0] <= 1 && 0 <= barycentric[1] && 0 <= barycentric[2]) {
		return triangle.apex * barycentric[0] + (triangle.apex + triangle.dir1) * barycentric[1] +
			   (triangle.apex + triangle.dir2) * barycentric[2];
	} else {
		return math_utils::closest_point_in_list({math_utils::Ray3d(triangle.apex, triangle.dir1).closest_point(p),
												  math_utils::Ray3d(triangle.apex, triangle.dir2).closest_point(p),},
												 p);
	}
}

Eigen::Vector3d cheat_away_from_vertices(const Eigen::Vector3d &p,
										 const Eigen::Vector3d &va,
										 const Eigen::Vector3d &vb,
										 const Eigen::Vector3d &vc,
										 const double margin) {

	if ((p - va).squaredNorm() < margin * margin) {
		return va + ((vb + vc) / 2.0 - va).normalized() * margin;
	} else if ((p - vb).squaredNorm() < margin * margin) {
		return vb + ((vc + va) / 2.0 - vb).normalized() * margin;
	} else if ((p - vc).squaredNorm() < margin * margin) {
		return vc + ((va + vb) / 2.0 - vc).normalized() * margin;
	} else {
		return p;
	}

}

Plane3d plane_from_points(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3) {

	const Eigen::Vector3d normal = (p1 - p2).cross(p3 - p2).normalized();

	return {normal, -p1.dot(normal)};

}

std::array<TriangleEdgeId, 2> edges_adjacent_to_vertex(TriangleVertexId vertex) {

	switch (vertex) {
		case VERTEX_A:
			return {EDGE_AB, EDGE_CA};
		case VERTEX_B:
			return {EDGE_BC, EDGE_AB};
		case VERTEX_C:
			return {EDGE_CA, EDGE_BC};
		default:
			throw std::runtime_error("Invalid vertex");
	}

}

std::array<TriangleVertexId, 2> vertices_in_edge(TriangleEdgeId edge) {
	switch (edge) {
		case EDGE_AB:
			return {VERTEX_A, VERTEX_B};
		case EDGE_BC:
			return {VERTEX_B, VERTEX_C};
		case EDGE_CA:
			return {VERTEX_C, VERTEX_A};
		default:
			throw std::runtime_error("Invalid edge id");
	}
}

Eigen::Vector3d
uniform_point_on_triangle(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3) {

	// Pick s,t from [0,1] using std::uniform_real_distribution
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	// Pick a point on a triangle
	const double r1 = sqrt(dis(gen));
	const double r2 = sqrt(dis(gen));

	return (1 - r1) * p1 + r1 * (1 - r2) * p2 + r1 * r2 * p3;
}

OctantIterator::OctantIterator(size_t i, const Eigen::AlignedBox3d &bounds) : i(i), total_box(bounds) {
}

OctantIterator::OctantIterator(const Eigen::AlignedBox3d &bounds) : i(0), total_box(bounds) {
}

OctantIterator OctantIterator::end() {
	return {8, Eigen::AlignedBox3d()};
}

OctantIterator &OctantIterator::operator++() {
	++i;
	return *this;
}

bool OctantIterator::operator==(const OctantIterator &other) const {
	return i == other.i && total_box.min() == other.total_box.min() && total_box.max() == other.total_box.max();
}

bool OctantIterator::operator!=(const OctantIterator &other) const {
	return !(*this == other);
}


Eigen::AlignedBox3d OctantIterator::operator*() const {

	Eigen::AlignedBox3d result;

	Eigen::Vector3d center = total_box.center();

	result.min().x() = (i & 1) ? total_box.min().x() : center.x();
	result.min().y() = (i & 2) ? total_box.min().y() : center.y();
	result.min().z() = (i & 4) ? total_box.min().z() : center.z();

	result.max().x() = (i & 1) ? center.x() : total_box.max().x();
	result.max().y() = (i & 2) ? center.y() : total_box.max().y();
	result.max().z() = (i & 4) ? center.z() : total_box.max().z();

	return result;

}

OctantIterator OctantIterator::operator++(int) {

	OctantIterator result = *this;
	++(*this);
	return result;

}

math_utils::Segment3d::Segment3d(const Eigen::Vector3d &start, const Eigen::Vector3d &end) : start(start), end(end) {
}

Eigen::Vector3d math_utils::Segment3d::closest_point(const Eigen::Vector3d &p) const {
	Eigen::ParametrizedLine<double, 3> ab(start, end - start);
	double t_ab = projectionParameter(ab, p);
	t_ab = std::clamp(t_ab, 0.0, 1.0);
	return ab.pointAt(t_ab);
}

Eigen::Vector3d math_utils::Segment3d::displacement() const {
	return end - start;
}

Eigen::ParametrizedLine<double, 3> math_utils::Segment3d::extended_line() const {
	return {start, displacement()};
}

math_utils::Ray3d::Ray3d(const Eigen::Vector3d &origin, const Eigen::Vector3d &direction)
		: origin(origin), direction(direction) {
}

Eigen::Vector3d math_utils::Ray3d::closest_point(const Eigen::Vector3d &p) const {
	Eigen::ParametrizedLine<double, 3> ab(origin, direction);
	double t_ab = projectionParameter(ab, p);
	t_ab = std::max(t_ab, 0.0);
	return ab.pointAt(t_ab);
}

EigenExt::ParametrizedLine3d math_utils::Ray3d::extended_line() const {
	return EigenExt::ParametrizedLine3d(origin, direction);
}

double math_utils::param_at_plane(const EigenExt::ParametrizedLine3d &p, int d, double value) {
	double dir_dim = p.direction()[d];
	if (dir_dim == 0.0) {
		return NAN;
	} else {
		double delta = value - p.origin()[d];
		return delta / dir_dim;
	}
}

bool math_utils::box_contains_segment(const Eigen::AlignedBox3d &box, const Segment3d &segment) {
	return box.contains(segment.start) && box.contains(segment.end);
}

std::optional<std::array<double, 2>>
math_utils::line_aabb_intersection_params(const Eigen::AlignedBox3d &box, const EigenExt::ParametrizedLine3d &segment) {

	double x_entry = param_at_plane(segment, 0, box.min().x());
	double x_exit = param_at_plane(segment, 0, box.max().x());
	if (x_entry > x_exit) {
		std::swap(x_entry, x_exit);
	}

	double y_entry = param_at_plane(segment, 1, box.min().y());
	double y_exit = param_at_plane(segment, 1, box.max().y());
	if (y_entry > y_exit) {
		std::swap(y_entry, y_exit);
	}

	double z_entry = param_at_plane(segment, 2, box.min().z());
	double z_exit = param_at_plane(segment, 2, box.max().z());
	if (z_entry > z_exit) {
		std::swap(z_entry, z_exit);
	}

	double entry = std::max({x_entry, y_entry, z_entry});
	double exit = std::min({x_exit, y_exit, z_exit});

	if (entry > exit) {
		return std::nullopt;
	} else {
		return std::array<double, 2>{entry, exit};
	}
}

bool math_utils::segment_intersects_aabb(const Eigen::AlignedBox3d &box, const Segment3d &segment) {

	auto intersections = line_aabb_intersection_params(box, segment.extended_line());

	if (!intersections) {
		return false;
	} else {
		return (*intersections)[0] >= 0.0 && (*intersections)[0] <= 1.0 && (*intersections)[1] >= 0.0 &&
			   (*intersections)[1] <= 1.0;
	}

}

OctantIterator math_utils::find_octant_containing_point(const Eigen::AlignedBox3d &box, const Eigen::Vector3d &point) {
	for (OctantIterator iter(box); iter != OctantIterator::end(); iter++) {
		if ((*iter).contains(point)) {
			return iter;
		}
	}
	assert(false && "Point not contained in any octant!");
}

bool math_utils::point_closer_to_center_than_edge(const Eigen::Vector3d &point, const Eigen::AlignedBox3d &box) {
	return (point - box.center()).norm() <= box.sizes()[0] * sqrt(3);
}

#if JSON_FUNCTIONS


Json::Value toJSON(const Eigen::Vector3d &v) {
	Json::Value json;
	json["x"] = v.x();
	json["y"] = v.y();
	json["z"] = v.z();
	return json;
}

Eigen::Vector3d fromJsonVector3d(const Json::Value &json) {
	return Eigen::Vector3d(
			json["x"].asDouble(),
			json["y"].asDouble(),
			json["z"].asDouble()
	);
}

Json::Value toJSON(const Eigen::Quaterniond &q) {
	Json::Value json;
	json["x"] = q.x();
	json["y"] = q.y();
	json["z"] = q.z();
	json["w"] = q.w();
	return json;
}

Eigen::Quaterniond fromJsonQuaternion3d(const Json::Value &json) {
	return {
			json["w"].asDouble(),
			json["x"].asDouble(),
			json["y"].asDouble(),
			json["z"].asDouble()
	};
}

Json::Value toJSON(const Eigen::Isometry3d &isom) {
	Json::Value json;
	json["translation"] = toJSON(isom.translation());
	json["orientation"] = toJSON(Eigen::Quaterniond(isom.rotation()));
	return json;
}

Eigen::Isometry3d fromJsonIsometry3d(const Json::Value &json) {

	Eigen::Isometry3d iso;
	iso.setIdentity();
	iso.translate(fromJsonVector3d(json["translation"]));
	iso.rotate(fromJsonQuaternion3d(json["orientation"]));
	return iso;
}

#endif

bool math_utils::intersects(const Eigen::AlignedBox3d &box, const Ray3d &ray3D) {
	auto intersections = line_aabb_intersection_params(box, ray3D.extended_line());

	if (!intersections) {
		return false;
	} else {
		return (*intersections)[1] >= 0.0;
	}
}

bool math_utils::hollow_sphere_intersects_hollow_aabb(const Eigen::Vector3d &sphere_center,
													  const double sphere_radius,
													  const Eigen::AlignedBox3d &aabb) {

	// Check if the boundary of the sphere is within the total_box of the node.
	// Algorithm from https://stackoverflow.com/a/41457896 (hollow sphere case)

	double dmin = 0; // Distance from the sphere center to the closest point on the AABB
	double dmax = 0; // Distance from the sphere center to the farthest point on the AABB
	bool face = false;

	double square_radius = std::pow(sphere_radius, 2);

	for (int dim = 0; dim < 3; dim++) {
		// Square distances between the sphere center and the extended planes of the AABB in this dimension
		double a = std::pow(sphere_center[dim] - aabb.min()[dim], 2);
		double b = std::pow(sphere_center[dim] - aabb.max()[dim], 2);

		// std::max(a, b) would be the greatest squared distance from the sphere center to the extended planes of the AABB in this dimension
		// So, we add this squared distance to the total, building up one component of the squared distance from the sphere center to the AABB
		// in every iteration. ( x^2 + y^2 + z^2, where each term gets added in a different iteration )
		dmax += std::max(a, b);


		if (sphere_center[dim] < aabb.min()[dim]) {
			face = true;
			dmin += a;
		} else if (sphere_center[dim] > aabb.max()[dim]) {
			face = true;
			dmin += b;
		} else if (std::min(a, b) <= square_radius)
			face = true;
	}

	return (face && (dmin <= square_radius) && (square_radius <= dmax));
}

math_utils::ViewPyramidFaces
math_utils::compute_view_pyramid_planes(const Eigen::Isometry3d &tf, double fovX, double fovY) {

	double x_slope = tan(fovX / 2);
	double y_slope = tan(fovY / 2);

	Eigen::Vector3d top_left = tf.rotation() * Eigen::Vector3d(-x_slope, 1.0, y_slope);
	Eigen::Vector3d top_right = tf.rotation() * Eigen::Vector3d(x_slope, 1.0, y_slope);
	Eigen::Vector3d bottom_left = tf.rotation() * Eigen::Vector3d(-x_slope, 1.0, -y_slope);
	Eigen::Vector3d bottom_right = tf.rotation() * Eigen::Vector3d(x_slope, 1.0, -y_slope);

	OpenTriangle left{tf.translation(), bottom_left, top_left};
	OpenTriangle right{tf.translation(), top_right, bottom_right};
	OpenTriangle top{tf.translation(), top_left, top_right};
	OpenTriangle bottom{tf.translation(), bottom_right, bottom_left};

	math_utils::ViewPyramidFaces faces{left, right, top, bottom};

	assert(faces.contains(tf * Eigen::Vector3d::UnitY()));

	return faces;
}

bool math_utils::intersects(const Eigen::AlignedBox3d &box, const Plane3d &plane) {

	// Compute the corners of the AABB.
	std::array<Eigen::Vector3d, 8> corners = {box.corner(Eigen::AlignedBox3d::BottomLeftFloor),
											  box.corner(Eigen::AlignedBox3d::BottomRightFloor),
											  box.corner(Eigen::AlignedBox3d::TopLeftFloor),
											  box.corner(Eigen::AlignedBox3d::TopRightFloor),
											  box.corner(Eigen::AlignedBox3d::BottomLeftCeil),
											  box.corner(Eigen::AlignedBox3d::BottomRightCeil),
											  box.corner(Eigen::AlignedBox3d::TopLeftCeil),
											  box.corner(Eigen::AlignedBox3d::TopRightCeil)};

	// Then we check if we find both a positive and a negative signed distance to the plane.

	bool positive = false;
	bool negative = false;

	for (const auto &corner: corners) {
		double signed_distance = plane.signedDistance(corner);
		if (signed_distance > 0) {
			positive = true;
		} else if (signed_distance < 0) {
			negative = true;
		}

		if (positive && negative) {
			return true;
		}
	}

	return false;

}

Eigen::Vector3d
math_utils::closest_point_in_list(std::initializer_list<Eigen::Vector3d> points, const Eigen::Vector3d &p) {

	assert(points.begin() != points.end());

	Eigen::Vector3d closest_point = *points.begin();
	double min_distance = (closest_point - p).squaredNorm();
	for (const auto &point: points) {
		double distance = (point - p).squaredNorm();
		if (distance < min_distance) {
			min_distance = distance;
			closest_point = point;
		}
	}

	return closest_point;
}

std::variant<std::monostate, double, Eigen::ParametrizedLine<double, 3>>
math_utils::find_intersection(const EigenExt::ParametrizedLine3d &segment, const Eigen::Hyperplane<double, 3> &plane) {

	// Check if the line is parallel to the plane.
	if (std::abs(segment.direction().dot(plane.normal())) < 1e-6) {
		// Check if the line is coplanar with the plane.
		if (std::abs(segment.origin().dot(plane.normal()) - plane.offset()) < 1e-6) {
			// The line is coplanar with the plane.
			return segment;
		} else {
			// The line is parallel to the plane, but not coplanar.
			return std::monostate();
		}
	} else {
		// The line is not parallel to the plane.

		double p_offset = -plane.signedDistance(segment.origin());

		double rate_of_approach = segment.direction().dot(plane.normal());

		return p_offset / rate_of_approach;
	}

}

std::variant<std::monostate, Eigen::Vector3d, math_utils::Segment3d>
math_utils::find_intersection(const math_utils::Segment3d &segment,
							  const Eigen::Hyperplane<double, 3> &plane,
							  double margin) {

	const Eigen::ParametrizedLine<double, 3> &parametrizedLine = segment.extended_line();
	const auto isect = find_intersection(parametrizedLine, plane);

	if (std::holds_alternative<std::monostate>(isect)) {
		// The line is parallel to the plane.
		return std::monostate();
	} else if (std::holds_alternative<Eigen::ParametrizedLine<double, 3>>(isect)) {
		// The line is coplanar with the plane.
		return segment;
	} else {
		// The line is not parallel to the plane.
		const double t = std::get<double>(isect);
		if (t >= -margin && t <= 1 + margin) {
			// The intersection point is on the line segment.
			return parametrizedLine.pointAt(t);
		} else {
			// The intersection point is not on the line segment.
			return std::monostate();
		}
	}


}

std::array<math_utils::Segment3d, 12> math_utils::aabb_edges(const Eigen::AlignedBox3d &box) {
	std::array<Segment3d, 12> edges{Segment3d(box.corner(Eigen::AlignedBox3d::BottomLeftFloor),
											  box.corner(Eigen::AlignedBox3d::BottomRightFloor)),
									Segment3d(box.corner(Eigen::AlignedBox3d::BottomRightFloor),
											  box.corner(Eigen::AlignedBox3d::TopRightFloor)),
									Segment3d(box.corner(Eigen::AlignedBox3d::TopRightFloor),
											  box.corner(Eigen::AlignedBox3d::TopLeftFloor)),
									Segment3d(box.corner(Eigen::AlignedBox3d::TopLeftFloor),
											  box.corner(Eigen::AlignedBox3d::BottomLeftFloor)),
									Segment3d(box.corner(Eigen::AlignedBox3d::BottomLeftCeil),
											  box.corner(Eigen::AlignedBox3d::BottomRightCeil)),
									Segment3d(box.corner(Eigen::AlignedBox3d::BottomRightCeil),
											  box.corner(Eigen::AlignedBox3d::TopRightCeil)),
									Segment3d(box.corner(Eigen::AlignedBox3d::TopRightCeil),
											  box.corner(Eigen::AlignedBox3d::TopLeftCeil)),
									Segment3d(box.corner(Eigen::AlignedBox3d::TopLeftCeil),
											  box.corner(Eigen::AlignedBox3d::BottomLeftCeil)),
									Segment3d(box.corner(Eigen::AlignedBox3d::BottomLeftFloor),
											  box.corner(Eigen::AlignedBox3d::BottomLeftCeil)),
									Segment3d(box.corner(Eigen::AlignedBox3d::BottomRightFloor),
											  box.corner(Eigen::AlignedBox3d::BottomRightCeil)),
									Segment3d(box.corner(Eigen::AlignedBox3d::TopRightFloor),
											  box.corner(Eigen::AlignedBox3d::TopRightCeil)),
									Segment3d(box.corner(Eigen::AlignedBox3d::TopLeftFloor),
											  box.corner(Eigen::AlignedBox3d::TopLeftCeil))};
	return edges;
}

std::variant<std::monostate, math_utils::Triangle3d, math_utils::Quad3d>
math_utils::find_intersection(const Eigen::AlignedBox3d &box, const Plane3d &plane) {

	// Find intersections between the plane and the AABB's edges.

	std::vector<Eigen::Vector3d> intersections;
	intersections.reserve(4);

	std::array<Segment3d, 12> edges = aabb_edges(box);

	for (const auto &edge: edges) {
		auto isect = find_intersection(edge, plane);
		if (std::holds_alternative<Eigen::Vector3d>(isect)) {
			std::cout << "Found intersection: " << std::get<Eigen::Vector3d>(isect).transpose() << std::endl;
			intersections.push_back(std::get<Eigen::Vector3d>(isect));
		}
	}

	switch (intersections.size()) {
		case 0:
			return std::monostate();
		case 1:
		case 2:
			throw std::runtime_error(
					"Plane does not intersect AABB in a way that can be represented as a triangle or quad.");
		case 3:
			return Triangle3d{intersections[0], intersections[1], intersections[2]};
		case 4:
			return Quad3d{intersections[0], intersections[1], intersections[2], intersections[3]};
		default:
			throw std::runtime_error("Unexpected number of intersections.");
	}

}

bool math_utils::ViewPyramidFaces::contains(const Eigen::Vector3d &point) const {
	return (point - left.apex).dot(*left.normal()) >= 0 && (point - right.apex).dot(*right.normal()) >= 0 &&
		   (point - top.apex).dot(*top.normal()) >= 0 && (point - bottom.apex).dot(*bottom.normal()) >= 0;
}

Eigen::Vector3d math_utils::ViewPyramidFaces::closest_point_on_any_plane(const Eigen::Vector3d &point) const {
	return closest_point_in_list({closest_point_on_open_triangle(point, left),
								  closest_point_on_open_triangle(point, right),
								  closest_point_on_open_triangle(point, top),
								  closest_point_on_open_triangle(point, bottom)}, point);
}
