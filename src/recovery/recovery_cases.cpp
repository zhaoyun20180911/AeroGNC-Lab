#include "gnc/recovery.hpp"

namespace gnc {

void applyRecoveryTestCase(RecoveryConfiguration& config, RecoveryTestCase testCase) {
    config.testCase = testCase;
    const NedFrame frame = makeLandingNedFrame(config.landingSite);
    Vec3 position = ecefPositionToNed(config.initialState.positionEcefM, frame);
    Vec3 velocity = ecefVectorToNed(config.initialState.velocityEcefMps, frame);
    config.vehicle.thrustBiasPercent = 0.0;
    config.atmosphereDensityScale = 1.0;
    config.windNedMps = {};
    switch (testCase) {
    case RecoveryTestCase::PositionError:
        position += Vec3{1200.0, -700.0, -250.0};
        break;
    case RecoveryTestCase::VelocityError:
        velocity += Vec3{20.0, -12.0, 8.0};
        break;
    case RecoveryTestCase::ThrustBias:
        config.vehicle.thrustBiasPercent = 3.0;
        break;
    case RecoveryTestCase::AtmosphereWind:
        config.atmosphereDensityScale = 1.12;
        config.windNedMps = {12.0, -18.0, 0.0};
        break;
    case RecoveryTestCase::Combined:
        position += Vec3{900.0, -600.0, -200.0};
        velocity += Vec3{15.0, -10.0, 6.0};
        config.vehicle.thrustBiasPercent = -3.0;
        config.atmosphereDensityScale = 1.10;
        config.windNedMps = {10.0, -15.0, 0.0};
        break;
    default:
        break;
    }
    config.initialState.positionEcefM = nedPositionToEcef(position, frame);
    config.initialState.velocityEcefMps = nedVectorToEcef(velocity, frame);
    config.initialState.bodyToEcef = attitudeFromBodyX(config.initialState.velocityEcefMps,
                                                       frame.eastEcef);
}

} // namespace gnc
