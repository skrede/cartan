#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_H

/// @file ik.h
/// @brief Convenience header for the inverse kinematics module.
///
/// Includes all IK types, solve policies, solver, and factory functions.

#include "cartan/serial/ik/ik_types.h"
#include "cartan/serial/ik/error_weight.h"
#include "cartan/serial/ik/limits_policy.h"
#include "cartan/serial/ik/ik_solve_policy.h"
#include "cartan/serial/ik/lm_solve_policy.h"
#include "cartan/serial/ik/dls_solve_policy.h"
#include "cartan/serial/ik/basic_ik_solver.h"
#include "cartan/serial/ik/default_solvers.h"
#include "cartan/serial/ik/slsqp_solve_policy.h"
#include "cartan/serial/ik/bobyqa_solve_policy.h"
#include "cartan/serial/ik/lbfgsb_solve_policy.h"
#include "cartan/serial/ik/cmaes_solve_policy.h"
#include "cartan/serial/ik/analytical_gradient.h"
#include "cartan/serial/ik/restart_solve_policy.h"
#include "cartan/serial/ik/halton_seed_generator.h"
#include "cartan/serial/ik/nw_sqp_solve_policy.h"
#include "cartan/serial/ik/nlopt_slsqp_solve_policy.h"
#include "cartan/serial/ik/nlopt_bobyqa_solve_policy.h"
#include "cartan/serial/ik/nablapp_lm_solve_policy.h"
#include "cartan/serial/ik/projected_lm_solve_policy.h"
#include "cartan/serial/ik/newton_raphson_solve_policy.h"
#include "cartan/serial/ik/nablapp_lbfgsb_solve_policy.h"
#include "cartan/serial/ik/augmented_lagrangian_solve_policy.h"

#endif
