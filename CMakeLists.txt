cmake_minimum_required(VERSION 3.24)

project(AerospaceGNC VERSION 3.2.0 LANGUAGES CXX)

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
)
target_include_directories(gnc_core PUBLIC include)
target_compile_definitions(gnc_core PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)

if(MSVC)
    target_compile_options(gnc_core PRIVATE /W4 /permissive- /utf-8)
endif()

add_executable(AerospaceGNC WIN32
    src/main.cpp
    src/settings_dialogs.cpp
    resources/AeroGNC.rc
)
target_link_libraries(AerospaceGNC PRIVATE gnc_core comctl32 comdlg32 dwmapi)
target_include_directories(AerospaceGNC PRIVATE resources)
if(MSVC)
    target_compile_options(AerospaceGNC PRIVATE /W4 /permissive- /utf-8)
endif()

add_custom_command(TARGET AerospaceGNC POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/data/rocket_missions"
            "$<TARGET_FILE_DIR:AerospaceGNC>/data/rocket_missions"
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

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT AerospaceGNC)
