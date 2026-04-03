#pragma once

#include <any>
#include <detours/detours.h>

namespace Hooks
{
    class BSFaceGenNiNodeHooks
    {
    public:
        static auto ApplyBoneLimitFix() -> void;

        static auto Hook() -> void
        {
            REL::Relocation SkinSingleGeometryCode1{
                REL::VariantID(26466, 27061, 0x03EBB30),
                REL::VariantOffset(0x108, 0x10F,
                                   0x108)}; // 0x03dc1c0, 0x03F6770, 0x03EBB30 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)

            //
            auto& trampoline = SKSE::GetTrampoline();
            SKSE::AllocTrampoline(14);

            //
            logger::debug("Applying BSFaceGenNiNodeHooks hooks!");

            //
            REL::Relocation BSFaceGenNiNode__vtbl{RE::VTABLE_BSFaceGenNiNode[0]};
            BSFaceGenNiNode__vtbl.write_vfunc(0x3E, SkinAllGeometry__Hook);

            //
            _SkinSingleGeometry = trampoline.write_call<5>(SkinSingleGeometryCode1.address(), SkinSingleGeometry__Hook);
            REL::safe_write(_SkinSingleGeometry.address() + REL::Offset(0x96).offset(), static_cast<uint8_t>(0x7));

            //
            ApplyBoneLimitFix();

            //
            logger::debug("...success");
        }

    public:
        static auto ProcessHeadPart(RE::BSFaceGenNiNode*, RE::BGSHeadPart*, RE::NiNode*, bool) -> void;
        static auto SkinAllGeometryCalls(RE::BSFaceGenNiNode*, RE::NiNode*, bool) -> void;
        static auto SkinSingleGeometry__Hook(RE::BSFaceGenNiNode*, RE::NiNode*, RE::BSGeometry*, bool) -> void;
        static auto SkinAllGeometry__Hook(RE::BSFaceGenNiNode*, RE::NiNode*, bool a_unk) -> void;
        static auto SkinAllGeometry(RE::BSFaceGenNiNode*, RE::NiNode*, bool a_unk) -> void;

        // Detour on SetBoneName entry so any caller (known or unknown) cannot write beyond FMD.bones[7].
        // Complements the loop-count cap in ApplyBoneLimitFix.
        static auto HookSetBoneName() -> void;
        static auto SetBoneName_Hook(RE::BSFaceGenModelExtraData*, std::uint32_t, RE::BSFixedString*) -> void;

        static inline REL::Relocation<decltype(&BSFaceGenNiNodeHooks::SkinSingleGeometry__Hook)> _SkinSingleGeometry;

    protected:
        using SetBoneName_t = decltype(SetBoneName_Hook);

    private:
        // BSFaceGenModelExtraData::SetBoneName
        static inline SetBoneName_t* _SetBoneName{nullptr};
    };

    class MainHooks
    {
    public:
        static auto Hook() -> void
        {
            REL::Relocation UpdateHook1{
                REL::VariantID(35551, 36544, 0x05B6D70),
                REL::VariantOffset(0x11F, 0x160,
                                   0x11F)}; // 0x05AF3D0, 0x05E7EE0, 0x05B6D70 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)
            REL::Relocation UpdateHook2{
                REL::VariantID(35565, 36564, 0x05BAB10),
                REL::VariantOffset(0x56D, 0x9DC,
                                   0x611)}; // 0x05B2FF0, 0x05EC240, 0x05BAB10 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)

            logger::debug("Applying MainHooks hooks!");

            //
            auto& trampoline = SKSE::GetTrampoline();

            //
            SKSE::AllocTrampoline(14);
            _Update = trampoline.write_call<5>(UpdateHook1.address(), Update);

            //
            SKSE::AllocTrampoline(14);
            _Unk_sub = trampoline.write_call<5>(UpdateHook2.address(), Unk_sub);

            //
            logger::debug("...success");
        }

        static auto Update(RE::Main*) -> void;

        static auto Unk_sub(std::any a_this) -> void; // RE::BSBethesdaPlatform*
    public:
        static inline REL::Relocation<decltype(&Unk_sub)> _Unk_sub;
        static inline REL::Relocation<decltype(&Update)> _Update;
    };

    class ActorEquipManagerHooks
    {
    public:
        static auto Hook() -> void
        {
            //
            logger::debug("Applying ActorEquipManagerHooks hooks!");

            //
            DetourAttach(reinterpret_cast<PVOID*>(&_func), reinterpret_cast<PVOID>(func));

            //
            logger::debug("...success");
        }

        static auto func(RE::ActorEquipManager*, RE::Actor*, RE::TESBoundObject*, RE::ExtraDataList*,
                         std::uint32_t, const RE::BGSEquipSlot*, bool, bool, bool, bool,
                         const RE::BGSEquipSlot*) -> bool;

    protected:
        using func_t = decltype(func);

    private:
        static inline func_t* _func{
            reinterpret_cast<func_t*>(REL::VariantID(37945, 38901, 0x06411A0)
                .address())}; // 0x0638190, 0x0670210, 0x06411A0 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)
    };

    class BipedAnimHooks
    {
    public:
        static inline std::vector<std::string> BackupNodes{};

    public:
        static auto Hook() -> void
        {
            logger::debug("Applying BipedAnimHooks hooks!");

            //
            DetourAttach(reinterpret_cast<PVOID*>(&_func), (PVOID)func);

            //
            logger::debug("...success");
        }

        static auto func(RE::BipedAnim*, RE::NiNode*, RE::BSFadeNode*, uint32_t, void*, void*,
                         void*) -> RE::NiAVObject*;

    protected:
        using func_t = decltype(func);

    private:
        static inline func_t* _func{
            (func_t*)REL::VariantID(15535, 15712, 0x01DB9E0)
            .address()}; // 0x01CAFB0, 0x01D83B0, 0x01DB9E0 (SE/1.5.97.0, AE/1.6.640.0, VR/1.4.15.0)
    };

    auto Install() -> void;
} // namespace Hooks
