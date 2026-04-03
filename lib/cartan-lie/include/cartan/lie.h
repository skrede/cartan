#ifndef HPP_GUARD_LIEPP_LIE_H
#define HPP_GUARD_LIEPP_LIE_H

/// @file lie.h
/// @brief Umbrella header for the liepp Lie group module.
///
/// Includes all Lie group types, frame wrappers, and utilities.

#include "liepp/types.h"
#include "liepp/detail/epsilon.h"

#include "liepp/lie/so2.h"
#include "liepp/lie/se2.h"
#include "liepp/lie/so3.h"
#include "liepp/lie/se3.h"
#include "liepp/lie/twist.h"
#include "liepp/lie/policy.h"
#include "liepp/lie/hat_vee.h"
#include "liepp/lie/axis_angle.h"
#include "liepp/lie/quaternion_utils.h"
#include "liepp/lie/se3_left_jacobian.h"

#include "liepp/frames/frames.h"
#include "liepp/frames/rotation.h"
#include "liepp/frames/transform.h"
#include "liepp/frames/framed_twist.h"
#include "liepp/frames/framed_wrench.h"

#endif
