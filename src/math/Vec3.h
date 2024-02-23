// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_VEC3_H
#define MGODPL_VEC3_H

#include <array>
#include <cmath>
#include <ostream>
#include <cassert>

namespace mgodpl::math {

	/**
	 * A simple 3D vector as an alternative to Vec3d, as per https://stackoverflow.com/a/70616903
	 */
	template<typename Scalar>
	struct Vec3 {

		std::array<Scalar, 3> components = {0, 0, 0};

		Vec3(Scalar x, Scalar y, Scalar z) : components({x, y, z}) {
		}

		explicit Vec3(const Scalar data[3]) : components({data[0], data[1], data[2]}) {} // NOLINT(*-avoid-c-arrays)

		explicit Vec3(const std::array<Scalar, 3> &components) : components(components) {}

		/**
		 * Add two vectors element-wise.
		 * @param other 		The other vector.
		 * @return 				The sum of the two vectors.
		 */
		Vec3 operator+(const Vec3 &other) const {
			return {x() + other.x(), y() + other.y(), z() + other.z()};
		}

		/**
		 * Subtract two vectors element-wise.
		 * @param other 		The other vector.
		 * @return 				The difference of the two vectors.
		 */
		Vec3 operator-(const Vec3 &other) const {
			return {x() - other.x(), y() - other.y(), z() - other.z()};
		}

		/**
		 * Multiply two vectors element-wise.
		 * @param other 		The other vector.
		 * @return 				The product of the two vectors.
		 */
		Vec3 operator*(const Vec3 &other) const {
			return {x() * other.x(), y() * other.y(), z() * other.z()};
		}

		/**
		 * Scalar multiplication.
		 */
		Vec3 operator*(const Scalar &scalar) const {
			return {x() * scalar, y() * scalar, z() * scalar};
		}

		/**
		 * Divide two vectors element-wise.
		 * @param other 		The other vector.
		 * @return 				The quotient of the two vectors.
		 */
		Vec3 operator/(const Vec3 &other) const {
			return {x() / other.x(), y() / other.y(), z() / other.z()};
		}

		void operator/=(const Scalar &scalar) {
			x() /= scalar;
			y() /= scalar;
			z() /= scalar;
		}

		void operator*=(const Scalar &scalar) {
			x() *= scalar;
			y() *= scalar;
			z() *= scalar;
		}

		/**
		 * Retrieve the x component of the vector.
		 */
		const Scalar &x() const {
			return components[0];
		}

		/**
		 * Retrieve the y component of the vector.
		 */
		const Scalar &y() const {
			return components[1];
		}

		/**
		 * Retrieve the z component of the vector.
		 */
		const Scalar &z() const {
			return components[2];
		}

		/**
		 * Retrieve the x component of the vector.
		 */
		Scalar &x() {
			return components[0];
		}

		/**
		 * Retrieve the y component of the vector.
		 */
		Scalar &y() {
			return components[1];
		}

		/**
		 * Retrieve the z component of the vector.
		 */
		Scalar &z() {
			return components[2];
		}

		/**
		 * Check whether this vector dominates another vector; i.e. whether all components of this vector are greater than or equal to the corresponding components of the other vector.
		 * @param other 		The other vector.
		 * @return 			True if this vector dominates the other vector, false otherwise.
		 */
		bool dominates(const Vec3 &other) const {
			return x() >= other.x() && y() >= other.y() && z() >= other.z();
		}

		/**
		 * Check whether this vector strictly dominates another vector; i.e. whether all components of this vector are greater than the corresponding components of the other vector.
		 */
		bool strictly_dominates(const Vec3 &other) const {
			return x() > other.x() && y() > other.y() && z() > other.z();
		}

		template<typename OtherScalar>
		Vec3<OtherScalar> cast() const {
			return {static_cast<OtherScalar>(x()), static_cast<OtherScalar>(y()), static_cast<OtherScalar>(z())};
		}

		/**
		 * < operator for lexicographical comparison.
		 */
		bool operator<(const Vec3 &other) const {
			return components < other.components;
		}

		static Vec3 UnitX() {
			return {1, 0, 0};
		}

		static Vec3 UnitY() {
			return {0, 1, 0};
		}

		static Vec3 UnitZ() {
			return {0, 0, 1};
		}

		static Vec3 Zero() {
			return {0, 0, 0};
		}

		/**
		 * Scalar division.
		 */
		[[nodiscard]] Vec3 operator/(const Scalar &scalar) const {
			return {x() / scalar, y() / scalar, z() / scalar};
		}

		/**
		 * Element-wise min.
		 */
		[[nodiscard]] Vec3 min(const Vec3 &other) const {
			return {std::min(x(), other.x()), std::min(y(), other.y()), std::min(z(), other.z())};
		}

		/**
		 * Element-wise max.
		 */
		[[nodiscard]] Vec3 max(const Vec3 &other) const {
			return {std::max(x(), other.x()), std::max(y(), other.y()), std::max(z(), other.z())};
		}

		/**
		 * Element access by index.
		 */
		const Scalar &operator[](size_t index) const {
			return components[index];
		}

		/**
		 * Element access by index.
		 */
		[[nodiscard]] Scalar &operator[](size_t index) {
			return components[index];
		}

		[[nodiscard]] Scalar squaredNorm() const {
			return x() * x() + y() * y() + z() * z();
		}

		[[nodiscard]] Scalar norm() const {
			return std::sqrt(squaredNorm());
		}

		/**
		 * Find the unit vector closest to this vector.
		 */
		[[nodiscard]] Vec3 nearestUnit() const {

			assert(squaredNorm() > 0.0);

			if (std::abs(x()) >= std::abs(y()) && std::abs(x()) >= std::abs(z())) {
				return Vec3(x() > 0 ? 1 : -1, 0, 0);
			} else if (std::abs(y()) >= std::abs(x()) && std::abs(y()) >= std::abs(z())) {
				return Vec3(0, y() > 0 ? 1 : -1, 0);
			} else {
				return Vec3(0, 0, z() > 0 ? 1 : -1);
			}
		}

		/**
		 * Get a pointer to the start of the underlying array.
		 */
		[[nodiscard]] const Scalar *data() const {
			return components.data();
		}

		[[nodiscard]] Scalar dot(Vec3 vec3) const {
			return x() * vec3.x() + y() * vec3.y() + z() * vec3.z();
		}

		/**
		 * Unary minus.
		 */
		[[nodiscard]] Vec3 operator-() const {
			return {-x(), -y(), -z()};
		}

		/**
		 * Stream output operator.
		 */
		friend std::ostream &operator<<(std::ostream &os, const Vec3 &vec3) {
			os << "(" << vec3.x() << ", " << vec3.y() << ", " << vec3.z() << ")";
			return os;
		}

		Vec3 <Scalar> normalized() const {
			return *this / norm();
		}

		void normalize() {
			*this /= norm();
		}

		[[nodiscard]] Vec3 cross(Vec3 other) const {
			return {
				y() * other.z() - z() * other.y(),
				z() * other.x() - x() * other.z(),
				x() * other.y() - y() * other.x()
			};
		}

		[[nodiscard]] Vec3 rounded() const {
			return {std::round(x()), std::round(y()), std::round(z())};
		}

		bool operator==(const Vec3 &other) const {
			return x() == other.x() && y() == other.y() && z() == other.z();
		}

		[[nodiscard]] double getX() const {
			return x();
		}

		[[nodiscard]] double getY() const {
			return y();
		}

		[[nodiscard]] double getZ() const {
			return z();
		}

		void setX(double x) {
			this->x() = x;
		}

		void setY(double y) {
			this->y() = y;
		}

		void setZ(double z) {
			this->z() = z;
		}

	};

	using Vec3d = Vec3<double>;
	using Vec3i = Vec3<int>;
}

#endif //MGODPL_VEC3_H
