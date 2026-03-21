#pragma once

#pragma warning(push)
#include <RE/Skyrim.h>
#include <REL/Relocation.h>
#include <SKSE/SKSE.h>

#ifdef NDEBUG
#	include <spdlog/sinks/basic_file_sink.h>
#else
#	include <spdlog/sinks/msvc_sink.h>
#endif
#pragma warning(pop)

//
#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <clocale>
#include <d3d11.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <ppl.h>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if defined(__has_include)
#	if __has_include(<amp.h>) && __has_include(<amp_graphics.h>) \
     && __has_include(<amp_math.h>) && __has_include(<amp_short_vectors.h>)
#		include <amp.h>
#		include <amp_graphics.h>
#		include <amp_math.h>
#		include <amp_short_vectors.h>
#	else
#		pragma message("C++ AMP headers not found — building without C++ AMP support")
// Minimal stubs for types you actually use from AMP can go here.
// Keep this small — add only the declarations your project needs.

#	endif
#else
#	include <amp.h>
#	include <amp_graphics.h>
#	include <amp_math.h>
#	include <amp_short_vectors.h>
#endif

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
}

// WRAPPER FUNCTIONS(makes the overall code cleaner as it's a function call instead of defineing RE::NiPointer<T>{...} / RE::BSTSmartPointer<T>{...} every time...
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
}

namespace std
{
    template <typename CharT>
    struct hash<RE::detail::BSFixedString<CharT>>
    {
        [[nodiscard]] auto operator()(const RE::detail::BSFixedString<CharT>& a_key) const noexcept -> std::size_t
        {
            return std::hash<const CharT*>{}(a_key.data());
        }
    };
}

#define DLLEXPORT __declspec(dllexport)

#include "Plugin.h"
