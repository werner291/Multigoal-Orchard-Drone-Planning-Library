// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9-2-24.
//

#include "scan_path_generators.h"

#include "JsonMeta.h"

namespace mgodpl
{
    std::vector<std::pair<std::string, std::vector<JsonMeta<ParametricPath>>>> gen_orbits(
        const math::Vec3d& fruit_center, const double target_radius)
    {
        std::vector<std::pair<std::string, std::vector<JsonMeta<mgodpl::ParametricPath>>>> orbit_types;

        {
            std::vector<mgodpl::JsonMeta<ParametricPath>> orbits;
            for (int i = 0; i <= 5; ++i)
            {
                double radius = target_radius + i * 0.2;
                Json::Value meta;
                meta["radius"] = radius;

                orbits.push_back(JsonMeta<ParametricPath>{
                    .meta = meta,
                    .data = fixed_radius_equatorial_orbit(fruit_center, radius)
                });
            }
            orbit_types.emplace_back("concentric_equatorial_orbits", orbits);
        }

        {
            std::vector<JsonMeta<ParametricPath>> orbits;
            for (int i = 0; i <= 5; ++i)
            {
                const double ascending_node_longitude = 0.0;
                double radius = target_radius + i * 0.2;
                Json::Value meta;
                meta["radius"] = radius;

                orbits.push_back(JsonMeta<ParametricPath>{
                    .meta = meta,
                    .data = polar_orbit(fruit_center, radius, ascending_node_longitude)
                });
            }
            orbit_types.emplace_back("concentric_polar_orbits", orbits);
        }

        {
            std::vector<JsonMeta<ParametricPath>> orbits;
            for (int j = 0; j <= 3; ++j)
            {
                double radius = target_radius + j * 0.1;
                for (int i = 1; i <= 3; ++i)
                {
                    int turns = i;
                    double height = i * 0.1;
                    Json::Value meta;
                    meta["radius"] = radius;
                    meta["turns"] = turns;
                    meta["height"] = height;

                    orbits.push_back(JsonMeta<ParametricPath>{
                        .meta = meta,
                        .data = helical_path(fruit_center, radius, turns, height)
                    });
                }
            }
            orbit_types.emplace_back("helical_orbits", orbits);
        }

        return orbit_types;
    }
}
