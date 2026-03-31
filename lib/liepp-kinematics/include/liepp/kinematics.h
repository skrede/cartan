#ifndef HPP_GUARD_LIEPP_KINEMATICS_H
#define HPP_GUARD_LIEPP_KINEMATICS_H

/// @file kinematics.h
/// @brief Umbrella header for the liepp kinematics module.
///
/// Includes chain, forward kinematics, Jacobian, velocity, and IK modules.

#include "liepp/chain/chain.h"
#include "liepp/chain/screw_axis.h"
#include "liepp/chain/joint_state.h"
#include "liepp/chain/joint_limits.h"
#include "liepp/chain/storage_trait.h"
#include "liepp/chain/kinematic_chain.h"

#include "liepp/kinematics/velocity.h"
#include "liepp/kinematics/jacobian.h"
#include "liepp/kinematics/fk_result.h"
#include "liepp/kinematics/kinematics.h"
#include "liepp/kinematics/forward_kinematics.h"

#include "liepp/ik/ik.h"

#endif
