function(cartan_dependency_is_exportable target out)
    if (NOT TARGET ${target})
        set(${out} TRUE PARENT_SCOPE)
        return()
    endif ()
    get_target_property(aliased ${target} ALIASED_TARGET)
    if (aliased)
        set(target ${aliased})
    endif ()
    get_target_property(imported ${target} IMPORTED)
    set(${out} ${imported} PARENT_SCOPE)
endfunction()

macro(cartan_assert_install_surface)
    set(_cartan_blockers "")
    cartan_dependency_is_exportable(Eigen3::Eigen _cartan_exportable)
    if (NOT _cartan_exportable)
        list(APPEND _cartan_blockers Eigen3)
    endif ()
    if (CARTAN_BUILD_URDF)
        cartan_dependency_is_exportable(pugixml::pugixml _cartan_exportable)
        if (NOT _cartan_exportable)
            list(APPEND _cartan_blockers pugixml)
        endif ()
    endif ()
    foreach (_cartan_backend IN LISTS CARTAN_OPTIONAL_BACKENDS)
        string(TOUPPER "${_cartan_backend}" _cartan_upper)
        if (CARTAN_BUILD_${_cartan_upper})
            cartan_dependency_is_exportable(${CARTAN_${_cartan_upper}_TARGET} _cartan_exportable)
            if (NOT _cartan_exportable)
                list(APPEND _cartan_blockers ${_cartan_backend})
            endif ()
        endif ()
    endforeach ()

    if (_cartan_blockers)
        list(JOIN _cartan_blockers ", " _cartan_blocker_text)
    endif ()
    if (CARTAN_ENABLE_INSTALL AND _cartan_blockers)
        message(FATAL_ERROR
            "cartan cannot generate install and export rules: ${_cartan_blocker_text} "
            "resolved to a target that is not imported, so install(EXPORT cartanTargets) would "
            "name a target that is a member of no export set. Configure with "
            "-DCARTAN_ENABLE_INSTALL=OFF to build without an install surface, or provide those "
            "dependencies as findable packages via CMAKE_PREFIX_PATH.")
    elseif (NOT CARTAN_ENABLE_INSTALL AND _cartan_blockers)
        message(STATUS "cartan: no install or export rules generated "
            "(CARTAN_ENABLE_INSTALL=OFF); ${_cartan_blocker_text} could not be exported anyway")
    elseif (NOT CARTAN_ENABLE_INSTALL)
        message(STATUS "cartan: no install or export rules generated "
            "(CARTAN_ENABLE_INSTALL=OFF), though every dependency is now exportable; "
            "reconfigure with -DCARTAN_ENABLE_INSTALL=ON to generate them")
    endif ()
endmacro()
