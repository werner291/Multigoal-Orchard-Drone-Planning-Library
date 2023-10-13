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

namespace mgodpl::math {

	/**
	 * A simple 3D vector as an alternative to Vec3d, as per https://stackoverflow.com/a/70616903
	 */
	template<typename Scalar>
	struct Vec3 {

		std::array<Scalar, 3> components = {0, 0, 0};

		Vec3(Scalar x, Scalar y, Scalar z) : components({x, y, z}) {
		}

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
		Vec3 operator/(const Scalar &scalar) const {
			return {x() / scalar, y() / scalar, z() / scalar};
		}

		/**
		 * Element-wise min.
		 */
		Vec3 min(const Vec3 &other) const {
			return {std::min(x(), other.x()), std::min(y(), other.y()), std::min(z(), other.z())};
		}

		/**
		 * Element-wise max.
		 */
		Vec3 max(const Vec3 &other) const {
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
		Scalar &operator[](size_t index) {
			return components[index];
		}

		Scalar squaredNorm() const {
			return x() * x() + y() * y() + z() * z();
		}

		Scalar norm() const {
			return std::sqrt(squaredNorm());
		}

		/**
		 * Find the unit vector closest to this vector.
		 */
		Vec3 nearestUnit() const {

			assert(squaredNorm() > 0.0);

			if (std::abs(x()) >= std::abs(y()) && std::abs(x()) >= std::abs(z())) {
				return {x() > 0 ? 1 : -1, 0, 0};
			} else if (std::abs(y()) >= std::abs(x()) && std::abs(y()) >= std::abs(z())) {
				return {0, y() > 0 ? 1 : -1, 0};
			} else {
				return {0, 0, z() > 0 ? 1 : -1};
			}
		}

		/**
		 * Get a pointer to the start of the underlying array.
		 */
		const Scalar *data() const {
			return components.data();
		}

		const Scalar dot(Vec3 vec3) {
			return x() * vec3.x() + y() * vec3.y() + z() * vec3.z();
		}
	};

	using Vec3d = Vec3<double>;
	using Vec3i = Vec3<int>;
}

#endif //MGODPL_VEC3_H
