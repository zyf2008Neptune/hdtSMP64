#pragma once

#pragma warning(push)
#include <RE/Skyrim.h>
#include <REL/Relocation.h>
#include <SKSE/SKSE.h>

#ifdef NDEBUG
#include <spdlog/sinks/basic_file_sink.h>
#else
#include <spdlog/sinks/msvc_sink.h>
#endif
#pragma warning(pop)

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <clocale>
#include <ppl.h>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std::literals;

namespace logger = SKSE::log;

namespace util
{
    using SKSE::stl::report_and_fail;
}

namespace RE
{
    template <class T1, class T2>
    [[nodiscard]] constexpr auto operator==(const BSTSmartPointer<T1>& a_lhs, T2* a_rhs) -> bool
    {
        return a_lhs.get() == a_rhs;
    }

    template <class T1, class T2>
    [[nodiscard]] constexpr auto operator!=(const BSTSmartPointer<T1>& a_lhs, T2* a_rhs) -> bool
    {
        return !(a_lhs.get() == a_rhs);
    }

    template <class T1, class T2>
    [[nodiscard]] constexpr auto operator==(const NiPointer<T1>& a_lhs, T2* a_rhs) -> bool
    {
        return a_lhs.get() == a_rhs;
    }

    template <class T1, class T2>
    [[nodiscard]] constexpr auto operator!=(const NiPointer<T1>& a_lhs, T2* a_rhs) -> bool
    {
        return !(a_lhs.get() == a_rhs);
    }
} // namespace RE

// WRAPPER FUNCTIONS(makes the overall code cleaner as it's a function call instead of defineing RE::NiPointer<T>{...} /
// RE::BSTSmartPointer<T>{...} every time...
namespace hdt
{
    template <class T>
    [[nodiscard]] auto make_nismart(T* a_ptr) -> RE::NiPointer<T>
    {
        RE::NiPointer<T> result;
        result.reset(a_ptr);
        return result;
    }

    template <class T>
    [[nodiscard]] auto make_smart(T* a_ptr) -> RE::BSTSmartPointer<T>
    {
        RE::BSTSmartPointer<T> result;
        result.reset(a_ptr);
        return result;
    }
} // namespace hdt

template <typename CharT>
struct std::hash<RE::detail::BSFixedString<CharT>>
{
    [[nodiscard]] auto operator()(const RE::detail::BSFixedString<CharT>& a_key) const noexcept -> std::size_t
    {
        if (a_key.empty())
        {
            return 0;
        }
        return std::hash<const CharT*>{}(a_key.data());
    }
}; // namespace std

#define DLLEXPORT __declspec(dllexport)

#include "Plugin.h"
