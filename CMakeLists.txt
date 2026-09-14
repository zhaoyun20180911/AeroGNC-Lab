cmake_minimum_required(VERSION 3.24)

project(AeroSysLab VERSION 4.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    # Produce a portable executable that does not require the VC++ Redistributable.
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

add_library(gnc_core
    src/math.cpp
    src/mission.cpp
    src/control.cpp
    src/disturbance.cpp
    src/orbit.cpp
    src/rocket_mission_sim.cpp
    src/satellite.cpp
    src/rocket.cpp
    src/recovery/recovery_coordinates.cpp
    src/recovery/recovery_defaults.cpp
    src/recovery/recovery_cases.cpp
    src/recovery/recovery_labels.cpp
    src/recovery/recovery_mission.cpp
    src/recovery/recovery_actuators.cpp
    src/recovery/recovery_grid_fin.cpp
    src/recovery/recovery_handoff.cpp
    src/recovery/recovery_qp_solver.cpp
    src/recovery/recovery_landing_guidance.cpp
    src/recovery/recovery_guidance.cpp
    src/recovery/recovery_simulation.cpp
    src/recovery/recovery_telemetry.cpp
)
target_include_directories(gnc_core PUBLIC include)
target_compile_definitions(gnc_core PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)

if(MSVC)
    target_compile_options(gnc_core PRIVATE /W4 /permissive- /utf-8)
endif()

add_executable(AeroSysLab WIN32
    src/main.cpp
    src/settings_dialogs.cpp
    resources/AeroGNC.rc
)
target_link_libraries(AeroSysLab PRIVATE gnc_core comctl32 comdlg32 dwmapi)
target_include_directories(AeroSysLab PRIVATE resources)
if(MSVC)
    target_compile_options(AeroSysLab PRIVATE /W4 /permissive- /utf-8)
endif()

add_custom_command(TARGET AeroSysLab POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/data/rocket_missions"
            "$<TARGET_FILE_DIR:AeroSysLab>/data/rocket_missions"
    COMMENT "Copying read-only rocket mission data"
)

enable_testing()
add_executable(gnc_tests tests/test_main.cpp)
target_link_libraries(gnc_tests PRIVATE gnc_core)
target_compile_definitions(gnc_tests PRIVATE
    GNC_TEST_DATA_DIR=L"${CMAKE_CURRENT_SOURCE_DIR}/data/rocket_missions")
if(MSVC)
    target_compile_options(gnc_tests PRIVATE /W4 /permissive- /utf-8)
endif()
add_test(NAME gnc_tests COMMAND gnc_tests)

add_executable(recovery_tests tests/recovery_test.cpp)
target_link_libraries(recovery_tests PRIVATE gnc_core)
if(MSVC)
    target_compile_options(recovery_tests PRIVATE /W4 /permissive- /utf-8)
endif()
add_test(NAME recovery_tests COMMAND recovery_tests)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT AeroSysLab)
