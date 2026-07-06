set(CARTAN_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")

# The UART driver moved from the monolithic `driver` component to the split
# `esp_driver_uart` in ESP-IDF 5.3; require whichever this toolchain ships.
if (IDF_VERSION_MAJOR GREATER 5 OR (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR GREATER_EQUAL 3))
    set(cartan_uart_component esp_driver_uart)
else ()
    set(cartan_uart_component driver)
endif ()

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES ${cartan_uart_component}
)

# Cartan's public include trees. SYSTEM keeps the strict warning surface from
# tripping on third-party Eigen + cartan template depth.
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "${CARTAN_ROOT}/lib/cartan-lie/include"
    "${CARTAN_ROOT}/lib/cartan-serial-chain/include"
    "${CARTAN_ROOT}/lib/cartan-analytical/include"
)

# Eigen lives outside ESP-IDF's default component graph; we add it through the
# IDF Component Manager (see ../idf_component.yml). If a local Eigen checkout
# is preferred, expose it via CARTAN_EIGEN_DIR.
if (DEFINED ENV{CARTAN_EIGEN_DIR})
    target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "$ENV{CARTAN_EIGEN_DIR}")
endif ()

# C++20 baseline matches the cartan project default.
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
