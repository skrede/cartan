set(CARTAN_MEIOS_PACKAGE meios)
set(CARTAN_MEIOS_TARGET meios::urdf)
set(CARTAN_MEIOS_REPOSITORY https://github.com/skrede/meios.git)
set(CARTAN_MEIOS_REVISION 890cf38e07c93384b41f14129b4551005cc7e8ed)

set(CARTAN_MEIOS_SOURCE_DIR "" CACHE PATH "Path to local meios source checkout")

option(CARTAN_MEIOS_ENABLE_PYTHON_EVAL
    "Build the description supplier's optional Python evaluation component" OFF)

# The benchmark suite reads its joint bounds off descriptions whose expressions
# call into the host language, which no other cartan configuration needs and the
# built-in evaluator cannot expand. Forced here rather than in benchmarks/,
# because the supplier is acquired under lib/ and is already made available by
# the time that directory is added.
if (CARTAN_BUILD_BENCHMARKS AND NOT CARTAN_MEIOS_ENABLE_PYTHON_EVAL)
    set(CARTAN_MEIOS_ENABLE_PYTHON_EVAL ON CACHE BOOL "" FORCE)
endif ()

macro(cartan_acquire_meios)
    if (CARTAN_MEIOS_SOURCE_DIR)
        FetchContent_Declare(meios
            SOURCE_DIR "${CARTAN_MEIOS_SOURCE_DIR}"
            EXCLUDE_FROM_ALL
            SYSTEM
        )
        # A local checkout produces an in-tree target belonging to no export
        # set, exactly as a fetch does. Reporting it as found would default the
        # install surface on, and the feasibility gate would then refuse the
        # configure outright instead of quietly generating no install rules.
        set(CARTAN_MEIOS_PROVIDER fetched)
    else ()
        # No version constraint anywhere: the revision below declares 0.2.0,
        # so a constraint written from the branch it sits on refuses a correct
        # installation at configure time. The revision is the only authority.
        #
        # GLOBAL, because acquisition happens in the URDF module's directory and
        # the install-surface feasibility gate runs at the top level: without it
        # the imported targets are scoped to this directory, the gate sees no
        # target at all, and an installed supplier is reported as unexportable.
        find_package(${CARTAN_MEIOS_PACKAGE} CONFIG QUIET GLOBAL)
        if (${CARTAN_MEIOS_PACKAGE}_FOUND OR TARGET ${CARTAN_MEIOS_TARGET})
            set(CARTAN_MEIOS_PROVIDER found)
        else ()
            # The clone is deliberately not shallow: a shallow fetch retrieves
            # only branch and tag tips, and the pin below is neither.
            FetchContent_Declare(meios
                GIT_REPOSITORY ${CARTAN_MEIOS_REPOSITORY}
                GIT_TAG ${CARTAN_MEIOS_REVISION}
                EXCLUDE_FROM_ALL
                SYSTEM
            )
            set(CARTAN_MEIOS_PROVIDER fetched)
        endif ()
    endif ()
    if (CARTAN_MEIOS_PROVIDER STREQUAL "fetched")
        block()
            set(MEIOS_BUILD_TESTS OFF)
            set(MEIOS_BUILD_TOOLS OFF)
            set(MEIOS_BUILD_EXAMPLES OFF)
            if (CARTAN_MEIOS_ENABLE_PYTHON_EVAL)
                set(MEIOS_BUILD_EVAL_PYTHON ${CARTAN_MEIOS_ENABLE_PYTHON_EVAL})
                # The supplier skips the component with a status line when the
                # embeddable interpreter is absent, which would leave a caller
                # that asked for it compiling against a header it links nothing
                # for.
                set(MEIOS_REQUIRE_EVAL_PYTHON ${CARTAN_MEIOS_ENABLE_PYTHON_EVAL})
            else ()
                set(MEIOS_BUILD_EVAL_PYTHON OFF)
            endif ()
            FetchContent_MakeAvailable(meios)
        endblock()
    endif ()
    cartan_report_meios_provider()
endmacro()

function(cartan_report_meios_provider)
    if (CARTAN_MEIOS_PROVIDER STREQUAL "found")
        set(origin "an installed package or an enclosing project's target")
    elseif (CARTAN_MEIOS_SOURCE_DIR)
        set(origin "the local checkout at ${CARTAN_MEIOS_SOURCE_DIR}")
    else ()
        set(origin "a clone of ${CARTAN_MEIOS_REPOSITORY} at ${CARTAN_MEIOS_REVISION}")
    endif ()
    # A fallback that silently overrides a prefix the user supplied on the
    # command line is indistinguishable from a resolution that worked, so the
    # chosen provider is reported whether or not anything went wrong.
    message(STATUS "cartan: robot description supplier ${CARTAN_MEIOS_PROVIDER} from ${origin}")
    if (NOT TARGET ${CARTAN_MEIOS_TARGET})
        message(FATAL_ERROR "the robot description supplier was ${CARTAN_MEIOS_PROVIDER} from "
            "${origin}, but ${CARTAN_MEIOS_TARGET} does not exist afterwards; the URDF module "
            "would compile against headers it links nothing for")
    endif ()
endfunction()
