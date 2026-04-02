#ifndef HPP_GUARD_LIEPP_SERIAL_IK_IK_H
#define HPP_GUARD_LIEPP_SERIAL_IK_IK_H

/// @file ik.h
/// @brief Convenience header for the inverse kinematics module.
///
/// Includes all IK types, solve policies, solver, and factory functions.

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/error_weight.h"
#include "liepp/serial/ik/limits_policy.h"
#include "liepp/serial/ik/ik_solve_policy.h"
#include "liepp/serial/ik/lm_solve_policy.h"
#include "liepp/serial/ik/dls_solve_policy.h"
#include "liepp/serial/ik/basic_ik_solver.h"
#include "liepp/serial/ik/default_solvers.h"
#include "liepp/serial/ik/slsqp_solve_policy.h"
#include "liepp/serial/ik/bobyqa_solve_policy.h"
#include "liepp/serial/ik/lbfgsb_solve_policy.h"
#include "liepp/serial/ik/cmaes_solve_policy.h"
#include "liepp/serial/ik/analytical_gradient.h"
#include "liepp/serial/ik/restart_solve_policy.h"
#include "liepp/serial/ik/halton_seed_generator.h"
#include "liepp/serial/ik/nw_sqp_solve_policy.h"
#include "liepp/serial/ik/nlopt_slsqp_solve_policy.h"
#include "liepp/serial/ik/nlopt_bobyqa_solve_policy.h"
#include "liepp/serial/ik/nablapp_lm_solve_policy.h"
#include "liepp/serial/ik/projected_lm_solve_policy.h"
#include "liepp/serial/ik/newton_raphson_solve_policy.h"
#include "liepp/serial/ik/nablapp_lbfgsb_solve_policy.h"
#include "liepp/serial/ik/augmented_lagrangian_solve_policy.h"

#endif
