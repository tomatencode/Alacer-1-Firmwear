#pragma once

#include <cstdint>

namespace buildConfig {

#if defined(ALACER_SIMULATION)
inline constexpr bool simulation = true;
#else
inline constexpr bool simulation = false;
#endif

#if defined(ALACER_SIMULATION_SPEED)
inline constexpr auto simulationSpeed = ALACER_SIMULATION_SPEED;
#else
inline constexpr auto simulationSpeed = 1;
#endif
static_assert(simulationSpeed >= 1 && simulationSpeed <= 100,
              "ALACER_SIMULATION_SPEED must be an integer from 1 to 100");
static_assert(static_cast<uint32_t>(simulationSpeed) == simulationSpeed,
              "ALACER_SIMULATION_SPEED must be an integer");

} // namespace buildConfig