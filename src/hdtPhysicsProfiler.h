#pragma once

#include <cstdint>

namespace hdt::physicsprofiler
{
    auto setCapture(bool a_enabled, std::uint64_t a_sampleFrames, std::uint64_t a_printFrames) -> void;
    auto advanceFrame() -> void;
} // namespace hdt::physicsprofiler
