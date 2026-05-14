#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_H

/// Umbrella header for the cartan serial chain module.
///
/// Includes chain, forward kinematics, Jacobian, velocity, and IK modules.

#include "cartan/serial/chain/chain.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/storage_trait.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/chain/static_chain.h"

#include "cartan/serial/fk/velocity.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/fk_result.h"
#include "cartan/serial/fk/kinematics.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/serial/ik/ik.h"

#endif
