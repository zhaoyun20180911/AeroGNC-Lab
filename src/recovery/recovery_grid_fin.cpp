#include "gnc/recovery.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {

GridFinAerodynamicOutput GridFinAerodynamicModel::evaluate(
    const GridFinAerodynamicInput& input,
    const RecoveryVehicleParameters& vehicle) const {
    GridFinAerodynamicOutput output{};
    if (!std::isfinite(input.dynamicPressurePa) || input.dynamicPressurePa <= 0.0) return output;
    const double limit = std::max(0.0, vehicle.gridFinLimitDeg * kDegToRad);
    std::array<double, 4> deflection = input.deflectionRad;
    for (double& value : deflection) value = clamp(value, -limit, limit);
    const double northCommand = 0.5 * (deflection[0] + deflection[2]);
    const double eastCommand = 0.5 * (deflection[1] + deflection[3]);
    const double compressibility = 1.0 / (1.0 + 0.04 * input.mach * input.mach);
    const double forceSlope = input.dynamicPressurePa * vehicle.gridFinAreaM2
                            * vehicle.gridFinLiftSlopePerRad * compressibility;
    output.forceNedN = {forceSlope * northCommand, forceSlope * eastCommand, 0.0};
    const double restoringPitch = -0.12 * forceSlope * input.angleOfAttackRad;
    const double restoringYaw = -0.12 * forceSlope * input.sideslipRad;
    output.momentBodyNm = {0.0, -output.forceNedN.x * 1.5 + restoringPitch,
                           output.forceNedN.y * 1.5 + restoringYaw};
    return output;
}

} // namespace gnc
