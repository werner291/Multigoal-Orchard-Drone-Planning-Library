//
// Created by werner on 9-2-24.
//

#ifndef JSONMETADATA_H
#define JSONMETADATA_H
#include <json/value.h>

namespace mgodpl
{
    /**
     * @brief A template struct used to pair up a piece of data with Json::Value metadata.
     *
     * This struct is used to store a piece of data of any type along with its associated metadata.
     * The metadata is stored as a Json::Value object. This pairing is useful for later analysis
     * where both the data and its metadata might be required.
     *
     * @tparam T The type of the data to be paired with the metadata.
     */
    template <typename T>
        struct JsonMeta
        {
            Json::Value meta; ///< The Json::Value object that holds the metadata.
            T data; ///< The piece of data to be paired with the metadata.
        };
    }

#endif //JSONMETADATA_H
