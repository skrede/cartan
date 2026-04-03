#ifndef HPP_GUARD_CARTAN_LIE_H
#define HPP_GUARD_CARTAN_LIE_H

/// @file lie.h
/// @brief Umbrella header for the cartan Lie group module.
///
/// Includes all Lie group types, frame wrappers, and utilities.

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/so2.h"
#include "cartan/lie/se2.h"
#include "cartan/lie/so3.h"
#include "cartan/lie/se3.h"
#include "cartan/lie/twist.h"
#include "cartan/lie/policy.h"
#include "cartan/lie/hat_vee.h"
#include "cartan/lie/axis_angle.h"
#include "cartan/lie/quaternion_utils.h"
#include "cartan/lie/se3_left_jacobian.h"

#include "cartan/frames/frames.h"
#include "cartan/frames/rotation.h"
#include "cartan/frames/transform.h"
#include "cartan/frames/framed_twist.h"
#include "cartan/frames/framed_wrench.h"

#endif
