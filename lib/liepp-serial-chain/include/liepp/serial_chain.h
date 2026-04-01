#ifndef HPP_GUARD_LIEPP_SERIAL_CHAIN_H
#define HPP_GUARD_LIEPP_SERIAL_CHAIN_H

/// @file serial_chain.h
/// @brief Umbrella header for the liepp serial chain module.
///
/// Includes chain, forward kinematics, Jacobian, velocity, and IK modules.

#include "liepp/serial/chain/chain.h"
#include "liepp/serial/chain/screw_axis.h"
#include "liepp/serial/chain/joint_tags.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/joint_limits.h"
#include "liepp/serial/chain/storage_trait.h"
#include "liepp/serial/chain/chain_concept.h"
#include "liepp/serial/chain/kinematic_chain.h"

#include "liepp/serial/fk/velocity.h"
#include "liepp/serial/fk/jacobian.h"
#include "liepp/serial/fk/fk_result.h"
#include "liepp/serial/fk/kinematics.h"
#include "liepp/serial/fk/forward_kinematics.h"

#include "liepp/serial/ik/ik.h"

#endif
