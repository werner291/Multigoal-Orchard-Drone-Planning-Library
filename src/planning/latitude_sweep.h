//
// Created by werner on 28-11-23.
//

#ifndef LATITUDE_SWEEP_H
#define LATITUDE_SWEEP_H

#include <vector>

#include "../math/Vec3.h"

namespace mgodpl
{
    struct Triangle {
        std::array<math::Vec3d, 3> vertices;
    };

    void latitude_sweep(const std::vector<Triangle>& triangles, const math::Vec3d& center);

    void longitude_sweep(const std::vector<Triangle>& triangles, const math::Vec3d& center);
}

#endif //LATITUDE_SWEEP_H
