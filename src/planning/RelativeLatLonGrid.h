//
// Created by werner on 17-1-24.
//

#ifndef RELATIVELATLONGRID_H
#define RELATIVELATLONGRID_H
#include "LatitudeLongitudeGrid.h"
#include "../math/Transform.h"

namespace mgodpl {

    struct RelativeLatLonGrid
    {
        LatLonGrid grid;
        math::Transformd transform;
        math::Transformd to_global;
    };

    RelativeLatLonGrid from_center_and_ideal_vector(const math::Vec3d& center, const math::Vec3d& ideal_vector);

    void insert_triangle(RelativeLatLonGrid& grid, const mgodpl::Triangle& triangle);
}

#endif //RELATIVELATLONGRID_H
