#pragma once

namespace hdt::physicsprofiler
{
    struct DumpOptions
    {
        double m_minScopeMs = 0.001;
        std::uint32_t m_maxDepth = 64;
        std::uint32_t m_flatLimit = 32;
        bool m_logTree = true;
        bool m_logFlat = true;
        bool m_logThreads = true;
    };

    auto install() -> void;
    auto uninstall() -> void;

    auto enter(const char* a_name) noexcept -> void;
    auto leave() noexcept -> void;

    auto endFrame() -> void;
    auto frameCountSinceReset() -> std::uint64_t;
    auto totalFrameCount() -> std::uint64_t;

    auto setProfileHistory(std::uint64_t a_frames) -> void;
    auto getProfileHistory() -> std::uint64_t;

    auto reset() -> bool;
    auto dumpAndReset(const DumpOptions& a_options = {}) -> bool;
    auto dumpEvery(std::uint64_t a_frames, const DumpOptions& a_options = {}) -> bool;
} // namespace hdt::physicsprofiler
