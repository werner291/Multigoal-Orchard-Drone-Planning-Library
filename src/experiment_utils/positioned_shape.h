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

		/**
		 * @brief Creates a PositionedShape with the provided shape and an identity transform.
		 *
		 * @param shape The shape of the geometry, such as a box or a mesh.
		 * @return A PositionedShape with the provided shape and an identity transform.
		 */
		static PositionedShape untransformed(const Shape& shape) {
			return PositionedShape{shape, math::Transformd::identity()};
		}

    };
}

#endif //POSITIONED_SHAPE_H
