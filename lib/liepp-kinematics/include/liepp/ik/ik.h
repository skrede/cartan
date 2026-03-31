#ifndef HPP_GUARD_LIEPP_IK_IK_H
#define HPP_GUARD_LIEPP_IK_IK_H

/// @file ik.h
/// @brief Convenience header for the inverse kinematics module.
///
/// Includes all IK types, solve policies, solver, and factory functions.

#include "liepp/ik/ik_types.h"
#include "liepp/ik/error_weight.h"
#include "liepp/ik/limits_policy.h"
#include "liepp/ik/ik_solve_policy.h"
#include "liepp/ik/lm_solve_policy.h"
#include "liepp/ik/dls_solve_policy.h"
#include "liepp/ik/basic_ik_solver.h"
#include "liepp/ik/default_solvers.h"
#include "liepp/ik/slsqp_solve_policy.h"
#include "liepp/ik/bobyqa_solve_policy.h"
#include "liepp/ik/lbfgsb_solve_policy.h"
#include "liepp/ik/analytical_gradient.h"
#include "liepp/ik/restart_solve_policy.h"
#include "liepp/ik/halton_seed_generator.h"
#include "liepp/ik/nlopt_slsqp_solve_policy.h"
#include "liepp/ik/nlopt_bobyqa_solve_policy.h"
#include "liepp/ik/projected_lm_solve_policy.h"
#include "liepp/ik/newton_raphson_solve_policy.h"

#endif
