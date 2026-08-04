set(CARTAN_MEIOS_PACKAGE meios)
set(CARTAN_MEIOS_TARGET meios::urdf)
set(CARTAN_MEIOS_REPOSITORY https://github.com/skrede/meios.git)
set(CARTAN_MEIOS_REVISION 5626542b01b9f141cbf2aa1bde0912ab96810a8b)

set(CARTAN_MEIOS_SOURCE_DIR "" CACHE PATH "Path to local meios source checkout")

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
        find_package(${CARTAN_MEIOS_PACKAGE} CONFIG QUIET)
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
            set(MEIOS_BUILD_EVAL_PYTHON OFF)
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
