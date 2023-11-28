//
// Created by werner on 28-11-23.
//

#ifndef POSITIONED_SHAPE_H
#define POSITIONED_SHAPE_H

#include "shapes.h"
#include "../math/Transform.h"

namespace mgodpl
{
    /**
         * @brief Geometry associated with a link; essentially a rigid body with a shape and a transform.
         */
    struct PositionedShape {

        /// The shape of the geometry, such as a box or a mesh.
        Shape shape;

        /// The transform of the geometry relative to the link's local frame of reference.
        math::Transformd transform;

    };
}

#endif //POSITIONED_SHAPE_H
