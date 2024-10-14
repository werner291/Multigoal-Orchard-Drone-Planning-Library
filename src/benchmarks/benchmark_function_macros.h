// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-2-24.
//

#ifndef BENCHMARK_FUNCTION_MACROS_H
#define BENCHMARK_FUNCTION_MACROS_H

#include <functional>
#include <map>
#include <string>
#include <filesystem>
#include <json/value.h>

// A top-level function that can be called to visualize something.
using BenchmarkFn = std::function<void(Json::Value &)>;

// A static map that maps a name to a visualization function.
extern std::map<std::string, BenchmarkFn> benchmarks;

#define REGISTER_BENCHMARK(name) \
    void name(Json::Value& results); \
    static bool is_##name##_registered = [](){ \
    benchmarks[#name] = name; \
    return true; \
    }(); \
    void name(Json::Value& results)

#endif //BENCHMARK_FUNCTION_MACROS_H
