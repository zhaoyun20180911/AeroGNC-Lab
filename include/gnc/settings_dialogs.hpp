#pragma once

#include "gnc/config.hpp"
#include "gnc/control.hpp"
#include "gnc/simulation.hpp"

#include <windows.h>

namespace gnc::gui {

bool showControlSettings(HWND owner, ScenarioKind scenario, ControlConfig& config,
                         const Vec3& inertiaDiagonalKgM2, const Vec3& actuatorAuthorityNm);

bool showDisturbanceSettings(HWND owner, ScenarioKind scenario,
                             SatelliteDisturbanceConfig& satellite,
                             RocketDisturbanceConfig& rocket);

bool showRocketGuidanceSettings(HWND owner, RocketGuidanceConfig& config);

bool showSatelliteOrbitControlSettings(HWND owner, SatelliteOrbitControlConfig& config);

bool showPerturbationSettings(HWND owner, PerturbationConfig& config);

} // namespace gnc::gui
