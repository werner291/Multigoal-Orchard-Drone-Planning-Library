#include <Eigen/Geometry>
#include <ompl/util/RandomNumbers.h>

/**
 * Given two vector4's, produce a third vector perpendicular to the inputs.
 * The result lies in the w=0 plane (Eigen::Vector4 is (x,y,z,w)).
 */
Eigen::Vector4d any_perpendicular_of_two(const Eigen::Vector4d& a, const Eigen::Vector4d& b) {

    // This is basically just the 3D vector cross product with a 0 tacked on the end.
    // w-component of the original vectors is ignored.
    return Eigen::Vector4d(
        a.y()*b.z() - b.y()*a.z(),
        b.x()*a.z() - a.x()*b.z(),
        a.x()*b.y() - b.x()*a.y(),
        0
    );
}

/**
 * Perform the generalized cross product where a 4-vector is produces which lies perpendicular
 * to the three input vectors, assuming all are nonzero and linearly independent.
 */
Eigen::Vector4d cross_three(const Eigen::Vector4d& u, const Eigen::Vector4d& v, const Eigen::Vector4d& t){
    // Source: https://www.researchgate.net/publication/318543243_Vector_Cross_Product_in_4D_Euclidean_Space_A_Maple_worksheet
    return Eigen::Vector4d(
        ( u.w() * v.z() - u.z() * v.w()) * t.y() + (-u.w() * v.y() + u.y() * v.w()) * t.z() + (-u.y() * v.z() + u.z() * v.y()) * t.w(),
        (-u.w() * v.z() + u.z() * v.w()) * t.x() + ( u.w() * v.x() - u.x() * v.w()) * t.z() + ( u.x() * v.z() - u.z() * v.x()) * t.w(),
        ( u.w() * v.y() - u.y() * v.w()) * t.x() + (-u.w() * v.x() + u.x() * v.w()) * t.y() + (-u.x() * v.y() + u.y() * v.x()) * t.w(),
        ( u.y() * v.z() - u.z() * v.y()) * t.x() + (-u.x() * v.z() + u.z() * v.x()) * t.y() + ( u.x() * v.y() - u.y() * v.x()) * t.z()
    );
}


Eigen::Quaterniond sampleInformedQuaternion(const Eigen::Quaterniond& qa,
                                            const Eigen::Quaterniond& qb,
                                            const double max_distance) {

    // Unfortunately, Eigen really only seems to consider Quaternions as useful
    // for rotations, so we convert to 4D vectors to get a richer API.
    Eigen::Vector4d
        ra(qa.x(), qa.y(), qa.z(), qa.w()),
        rb(qb.x(), qb.y(), qb.z(), qb.w());

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

    if (max_distance+between_inputs >= 2.0*M_PI) {
        // Special case: the tolerance is so large that any orientation will fulfill the requirement,
        // even if the resulting movement means wrapping all the way around the back of the sphere.
        result_vec = Eigen::Vector4d(rng.gaussian01(),
                rng.gaussian01(),
                rng.gaussian01(),
                rng.gaussian01());
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
        * For the relationship between spherical ellipses and regular ellipses, see this thesis: https://www.geometrie.tuwien.ac.at/theses/pdf/diplomarbeit_tranacher.pdf
        */

        // Compute the semi-major and semi-minor axes of this ellipse.
        // Note: the ellipse is stretched only in one dimension.
        double sma = std::tan(max_distance / 4.0);
        double smi = std::tan(std::acos(std::cos(max_distance / 2.0) / std::cos(between_inputs / 2.0))/2.0);

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
                (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z())
        );

        // Absolute value in the angle calculations makes it so that there is actually
        // an area on the exact opposite of the unit hypersphere that is also a valid
        // sampling region of the same size. We pick from there with 50% probability.
        if (rng.uniformBool()) {
            sample.z() *= -1.0;
        }

        // Sanity check: did the inverse-projection actually project he sample into the sphere?
        assert(abs(on_sphere.norm()-1.0) < 1.0e-10);

        // Now, we need to "rotate" around the hypersphere such that the projection
        // of the ellipsoid is actually centered on the input quaternions.

        // A vector pointing in the direction from one point to the other.
        // the ellipsoid is stretched along this direction.
        auto fw = (ra - rb).normalized();

        // A normal vector of the hypersphere at a point halfway between the two quaternions.
        auto up = (0.5 * ra + 0.5 * rb).normalized();
        // Then, we obtain two more perpendicular 4-vectors to build a local coordinate frame.
        Eigen::Vector4d p1 = any_perpendicular_of_two(up,fw).normalized();
        Eigen::Vector4d p2 = cross_three(up,fw,p1);

        // Assemble the vectors into a matrix
        Eigen::Matrix4d xf {
                {fw.x(),p1.x(),p2.x(),up.x()},
                {fw.y(),p1.y(),p2.y(),up.y()},
                {fw.z(),p1.z(),p2.z(),up.z()},
                {fw.w(),p1.w(),p2.w(),up.w()},
        };

        // Use the matrix to transform our sample into the coordinate frame centered onto the two quaternions.
        result_vec = xf * on_sphere;
    }

    // Convert the result 1-to-1 to a quaternion, and return it.
    return Eigen::Quaterniond(result_vec.w(),result_vec.x(),result_vec.y(),result_vec.z());
}
