#pragma once

/*
 * C++ filter component aggregate header.
 *
 * Include this file when a module needs several filter types:
 *
 *   #include "component/filter/filter.hpp"
 *
 *   auto lp = mr::comp::filter::low_pass::Build(30.0f, 1000.0f);
 *   auto hp = mr::comp::filter::high_pass::Build(2.0f, 1000.0f);
 *   auto nt = mr::comp::filter::notch::Build(50.0f, 5.0f, 1000.0f);
 *   auto kf = mr::comp::filter::kalman::Build(0.01f, 0.1f);
 */

#include "component/filter/filter_types.hpp"
#include "component/filter/high_pass_filter.hpp"
#include "component/filter/kalman_filter.hpp"
#include "component/filter/low_pass_filter.hpp"
#include "component/filter/notch_filter.hpp"
