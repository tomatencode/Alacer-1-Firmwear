#pragma once

#include <cstdint>

namespace buildConfig {

#if defined(ALACER_SIMULATION)
inline constexpr bool simulation = true;
#else
inline constexpr bool simulation = false;
#endif

} // namespace buildConfig