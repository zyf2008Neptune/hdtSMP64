#pragma once

#include "DynamicHDT.h"

namespace hdt::papyrus
{
    auto RegisterAllFunctions(const SKSE::PapyrusInterface* a_papy_intfc) -> bool;

    auto ReloadPhysicsFile(RE::StaticFunctionTag* base, RE::Actor* on_actor, RE::TESObjectARMA* on_item,
                           RE::BSFixedString physics_file_path, bool persist, bool verbose_log) -> bool;

    auto SwapPhysicsFile(RE::StaticFunctionTag* base, RE::Actor* on_actor, RE::BSFixedString old_physics_file_path,
                         RE::BSFixedString new_physics_file_path, bool persist, bool verbose_log) -> bool;

    auto QueryCurrentPhysicsFile(RE::StaticFunctionTag* base, RE::Actor* on_actor, RE::TESObjectARMA* on_item,
                                 bool verbose_log) -> RE::BSFixedString;

    // Toggle bones between kinematic and dynamic
    // Returns array of bools, each entry is the PREVIOUS state of that bone index:
    // - true  = bone was dynamic (physics was ON)
    // - false = bone was kinematic (physics was OFF / not found)
    auto TogglePhysics(RE::StaticFunctionTag* base, const RE::Actor* actor, std::vector<RE::BSFixedString> boneNames,
                       bool on) -> std::vector<bool>;

    // Reset an actor's SMP physics systems
    // - full = true  -> complete reset, bones snap to reference pose
    // - full = false -> soft reset, current bone poses are preserved
    auto ResetPhysics(RE::StaticFunctionTag* base, RE::Actor* actor, bool full) -> void;

    namespace impl
    {
        auto ReloadPhysicsFileImpl(uint32_t on_actor_formID, uint32_t on_item_formID,
                                   std::string_view physics_file_path, bool persist, bool verbose_log) -> bool;

        auto SwapPhysicsFileImpl(uint32_t on_actor_formID, std::string_view old_physics_file_path,
                                 std::string_view new_physics_file_path, bool persist, bool verbose_log) -> bool;

        auto QueryCurrentPhysicsFileImpl(uint32_t on_actor_formID, uint32_t on_item_formID, bool verbose_log)
            -> std::string;

        auto TogglePhysicsImpl(const RE::Actor* actor, const std::vector<RE::BSFixedString>& boneNames, bool on)
            -> std::vector<bool>;

        auto ResetPhysicsImpl(const RE::Actor* actor, bool full) -> void;
    } // namespace impl

    // uint32_t FindOrCreateAnonymousSystem(RE::StaticFunctionTag* base, RE::TESObjectARMA* system_model, bool
    // verbose_log);

    // uint32_t AttachAnonymousSystem(RE::StaticFunctionTag* base, RE::Actor* on_actor, uint32_t system_handle, bool
    // verbose_log);

    // uint32_t DetachAnonymousSystem(RE::StaticFunctionTag* base, RE::Actor* on_actor, uint32_t system_handle, bool
    // verbose_log);
} // namespace hdt::papyrus
