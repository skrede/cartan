#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_H

/// @file ik.h
/// @brief Convenience header for the inverse kinematics module.
///
/// Includes all IK types, solve policies, solver, and factory functions.

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/ik_result.h"
#include "cartan/serial/ik/basic_ik_runner.h"
#include "cartan/serial/ik/ik_validation.h"
#include "cartan/serial/ik/solvers.h"

#include "cartan/serial/ik/solver/dls.h"
#include "cartan/serial/ik/solver/lm.h"
#include "cartan/serial/ik/solver/lbfgsb.h"
#include "cartan/serial/ik/solver/projected_lm.h"
#include "cartan/serial/ik/solver/newton_raphson.h"
#include "cartan/serial/ik/solver/exhaustive_ik_runner.h"

#include "cartan/serial/ik/wrapper/restart_wrapper.h"

#include "cartan/serial/ik/concepts/solve_concept.h"

#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/policy/error_weight.h"

#ifdef CARTAN_BUILD_ARGMIN
#include "cartan/serial/ik/solver/cmaes.h"
#include "cartan/serial/ik/solver/argmin_lm.h"
#include "cartan/serial/ik/solver/argmin_lbfgsb.h"
#include "cartan/serial/ik/solver/argmin_slsqp.h"
#include "cartan/serial/ik/solver/argmin_bobyqa.h"
#include "cartan/serial/ik/solver/nw_sqp.h"
#include "cartan/serial/ik/solver/augmented_lagrangian.h"
#endif

#ifdef CARTAN_HAS_NLOPT
#include "cartan/serial/ik/solver/nlopt_slsqp.h"
#include "cartan/serial/ik/solver/nlopt_bobyqa.h"
#endif

#endif
