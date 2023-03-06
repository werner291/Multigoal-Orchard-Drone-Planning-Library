// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_EIGENEXT_H
#define NEW_PLANNERS_EIGENEXT_H

#include <Eigen/src/Geometry/Hyperplane.h>
#include <Eigen/src/Geometry/Quaternion.h>

namespace EigenExt {

	using Plane3d = Eigen::Hyperplane<double, 3>;

	/**
	 * Small class that ensures that the vector is always unit-length.
	 * @tparam N 		Dimension of the vector
	 * @tparam D 		Scalar type
	 */
	template<size_t N, typename D>
	class UnitVector {

		const double CHECK_PRECISION = 1e-6;

		using Vector = Eigen::Matrix<D, N, 1>;

		Vector vector;

	public:

		/**
		 * @brief Get the underlying vector
		 */
		inline const Vector &getVector() const {
			return vector;
		}

		inline const Vector &operator*() const {
			return vector;
		}

		inline const Vector *operator->() const {
			return &vector;
		}

		inline UnitVector<3, double> operator-() const {
			return UnitVector<3, double>(-vector);
		}

		inline UnitVector &operator=(const UnitVector &v) {
			vector = v.vector;
			return *this;
		}

		/**
			 * Construct a unit vector from a vector, asserting that the vector is unit-length.
			 * @param vector 		Vector to construct from
			 */
		inline explicit UnitVector(Vector vector) : vector(vector) {
			assert(std::abs(vector.norm() - 1.0) < CHECK_PRECISION);
		}

		// Copy constructor
		inline UnitVector(const UnitVector &other) : vector(other.vector) {
		}

		// Casting
		inline operator const Vector &() const { // NOLINT(google-explicit-constructor)
			return vector;
		}
	};

	using UVector1d = UnitVector<1, double>;
	using UVector2d = UnitVector<2, double>;
	using UVector3d = UnitVector<3, double>;

	using UVector1f = UnitVector<1, float>;
	using UVector2f = UnitVector<2, float>;
	using UVector3f = UnitVector<3, float>;

	template<typename D>
	UnitVector<3, D> operator*(const Eigen::Quaterniond &q, const UnitVector<3, D> &v) {
		return UnitVector<3, D>(q * v.getVector());
	}

	using ParametrizedLine3d = Eigen::ParametrizedLine<double, 3>;

}

#endif //NEW_PLANNERS_EIGENEXT_H
