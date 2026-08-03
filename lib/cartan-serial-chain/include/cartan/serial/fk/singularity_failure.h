#ifndef HPP_GUARD_CARTAN_SERIAL_FK_SINGULARITY_FAILURE_H
#define HPP_GUARD_CARTAN_SERIAL_FK_SINGULARITY_FAILURE_H

namespace cartan
{

/// Why a singularity measure has no answer. One name per case across the whole
/// analysis surface: a caller that receives no measure can tell a chain with no
/// joints from a configuration that produced no Jacobian, which a single shared
/// absence cannot say.
enum class singularity_failure
{
    empty_spectrum,         ///< The chain has no joints, so there is no spectrum to measure.
    zero_spectrum,          ///< The largest singular value is not positive -- an entirely zero Jacobian -- so no ratio against it is defined.
    invalid_configuration,  ///< The configuration produced no Jacobian: a joint vector of the wrong length, or one carrying a non-finite component.
    invalid_length          ///< The characteristic length is not a positive finite value, so it cannot divide the Jacobian's linear rows.
};

/// Human-readable diagnostic for a singularity_failure, for logging and binding
/// exception messages. Returns a static string literal; no allocation.
constexpr const char* message(singularity_failure failure)
{
    switch (failure)
    {
    case singularity_failure::empty_spectrum:
        return "Chain has no joints, so there is no spectrum to measure";
    case singularity_failure::zero_spectrum:
        return "Jacobian is entirely zero, so no ratio of its singular values is defined";
    case singularity_failure::invalid_configuration:
        return "Configuration does not produce a Jacobian";
    case singularity_failure::invalid_length:
        return "Characteristic length is not a positive finite value";
    }
    return "Unknown singularity_failure";
}

/// Whether the failure is the caller's mistake rather than a measure that is
/// undefined at a well-formed question. A binding layer raises for the first and
/// answers an absence for the second, and a switch with no default arm makes a
/// value added later a compile error here rather than a silent classification.
constexpr bool is_invalid_argument(singularity_failure failure)
{
    switch (failure)
    {
    case singularity_failure::empty_spectrum:
    case singularity_failure::zero_spectrum:
        return false;
    case singularity_failure::invalid_configuration:
    case singularity_failure::invalid_length:
        return true;
    }
    return true;
}

}

#endif
