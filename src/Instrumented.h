// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-3-23.
//

#ifndef NEW_PLANNERS_INSTRUMENTED_H
#define NEW_PLANNERS_INSTRUMENTED_H

#include <memory>
#include <json/value.h>
#include <functional>

template<typename Base>
class Instrumented {

	std::unique_ptr<Base> base;

	std::function<Json::Value()> parameters;

};

#endif //NEW_PLANNERS_INSTRUMENTED_H
