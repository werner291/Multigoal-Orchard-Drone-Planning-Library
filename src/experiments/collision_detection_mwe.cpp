#include <chrono>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_request.h>
#include <random_numbers/random_numbers.h>

int main(int argc, char** argv)
{
    random_numbers::RandomNumberGenerator rng(42);

    for (const double radius: {0.5, 1.0, 2.0, 5.0, 10.0})
    {
        for (const int n_triangles: { 10, 100, 1000, 10000, 100000 })
        {
            std::shared_ptr<fcl::BVHModel<fcl::OBBd>> without_RSS = std::make_shared<fcl::BVHModel<fcl::OBBd>>();
            std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> with_RSS = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();

            without_RSS->beginModel();
            with_RSS->beginModel();

            for (size_t i = 0; i < n_triangles; ++i)
            {
                fcl::Vector3d a(rng.uniformReal(radius, radius), rng.uniformReal(radius, radius), rng.uniformReal(radius, radius));
                fcl::Vector3d b(rng.uniformReal(radius, radius), rng.uniformReal(radius, radius), rng.uniformReal(radius, radius));
                fcl::Vector3d c(rng.uniformReal(radius, radius), rng.uniformReal(radius, radius), rng.uniformReal(radius, radius));

                without_RSS->addTriangle(a, b, c);
                with_RSS->addTriangle(a, b, c);
            }

            with_RSS->endModel();
            without_RSS->endModel();

            fcl::CollisionObjectd co_without(without_RSS, fcl::Transform3d::Identity());
            fcl::CollisionObjectd co_with(with_RSS, fcl::Transform3d::Identity());

            const auto box = std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0);

            size_t N_SAMPLES = 1000;

            long time_without = 0;
            long time_with = 0;

            for (size_t sample_i = 0; sample_i < N_SAMPLES; ++sample_i)
            {

                // Generate a random transform.
                fcl::Transform3d tf = fcl::Translation3d(rng.uniformReal(-radius, radius), rng.uniformReal(-radius, radius), rng.uniformReal(-radius, radius)) *
                    fcl::Quaterniond(rng.uniformReal(-1.0, 1.0), rng.uniformReal(-1.0, 1.0), rng.uniformReal(-1.0, 1.0), rng.uniformReal(-1.0, 1.0)).normalized();

                fcl::CollisionObjectd co(box, tf);

                bool c1 = false;
                bool c2 = false;

                auto time_1 = std::chrono::high_resolution_clock::now();
                {
                    fcl::CollisionRequestd req;
                    fcl::CollisionResultd res;
                    fcl::collide(&co_without, &co, req, res);

                    c1 = res.isCollision();
                }
                auto time_2 = std::chrono::high_resolution_clock::now();
                {
                    fcl::CollisionRequestd req;
                    fcl::CollisionResultd res;
                    fcl::collide(&co_with, &co, req, res);

                    c2 = res.isCollision();
                }
                auto time_3 = std::chrono::high_resolution_clock::now();

                time_without += std::chrono::duration_cast<std::chrono::nanoseconds>(time_2 - time_1).count();
                time_with += std::chrono::duration_cast<std::chrono::nanoseconds>(time_3 - time_2).count();

                assert(c1 == c2);

            }

            std::cout << "Radius: " << radius << std::endl;
            std::cout << "N triangles: " << n_triangles << std::endl;
            std::cout << "Time without RSS: " << time_without << std::endl;
            std::cout << "Time with RSS:    " << time_with << std::endl;
            std::cout << "Nodes in OBB tree " << without_RSS->getNumBVs() << std::endl;
            std::cout << "Nodes in OBBRSS tree " << with_RSS->getNumBVs() << std::endl;
            std::cout << "Ratio: " << (double)time_without / (double)time_with << std::endl;
            std::cout << std::endl;
        }
    }

}
