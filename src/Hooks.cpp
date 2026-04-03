#include "ActorManager.h"

#include "Events.h"
#include "Hooks.h"

//
#include <xbyak/xbyak.h>

#include <utility>

namespace Hooks
{
    auto BSFaceGenNiNodeHooks::ProcessHeadPart(RE::BSFaceGenNiNode* const a_this, RE::BGSHeadPart* headPart,
                                               RE::NiNode* a_skeleton, const bool a_unk) -> void
    {
        //
        if (headPart)
        {
            if (const auto headNode = a_this->GetObjectByName(headPart->formEditorID))
            {
                if (const auto headGeometry = headNode->AsGeometry())
                {
                    SkinSingleGeometry__Hook(a_this, a_skeleton, headGeometry, a_unk);
                }
            }

            //
            for (const auto it : headPart->extraParts)
            {
                ProcessHeadPart(a_this, it, a_skeleton, a_unk);
            }
        }
    }

    auto BSFaceGenNiNodeHooks::SkinAllGeometryCalls(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton,
                                                    const bool a_unk) -> void
    {
        auto needRegularCall = true;
        const auto userData = a_skeleton->GetUserData();
        if (hdt::ActorManager::instance()->skeletonNeedsParts(a_skeleton) && userData)
        {
            RE::TESForm* form = RE::TESForm::LookupByID(userData->formID);
            if (const RE::Actor* actor = skyrim_cast<RE::Actor*>(form))
            {
                auto actorBase = skyrim_cast<RE::TESNPC*>(actor->data.objectReference);
                uint32_t numHeadParts;
                RE::BGSHeadPart** Headparts;

                if (actorBase->HasOverlays())
                {
                    numHeadParts = actorBase->GetNumBaseOverlays();
                    Headparts = actorBase->GetBaseOverlays();
                }
                else
                {
                    numHeadParts = static_cast<std::uint8_t>(actorBase->numHeadParts);
                    Headparts = actorBase->headParts;
                }

                if (Headparts)
                {
                    for (uint32_t i = 0; i < numHeadParts; i++)
                    {
                        if (Headparts[i])
                        {
                            ProcessHeadPart(a_this, Headparts[i], a_skeleton, a_unk);
                        }
                    }
                }

                if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->formID == 0x14)
                {
                    needRegularCall = false;
                }
            }
        }

        if (needRegularCall)
        {
            SkinAllGeometry(a_this, a_skeleton, a_unk);
        }
    }

    auto BSFaceGenNiNodeHooks::SkinSingleGeometry__Hook(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton,
                                                        RE::BSGeometry* a_triShape, [[maybe_unused]] bool a_unk) -> void
    {
        //
        auto name = "";
        uint32_t formId = 0x0;

        //
        if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->GetBaseObject())
        {
            if (const auto bname = skyrim_cast<RE::TESFullName*>(a_skeleton->GetUserData()->GetBaseObject()))
            {
                name = bname->GetFullName();
            }

            if (const auto bnpc = skyrim_cast<RE::TESNPC*>(a_skeleton->GetUserData()->GetBaseObject());
                bnpc && bnpc->faceNPC)
            {
                formId = bnpc->faceNPC->formID;
            }
        }

        //
        logger::debug("SkinSingleGeometry {} {} - {}, {}, (formid {:08x} base form {:08x} head template form {:08x})",
                      a_skeleton->name.c_str(), a_skeleton->GetChildren().size(), a_triShape->name.c_str(), name,
                      a_skeleton->GetUserData() ? a_skeleton->GetUserData()->formID : 0x0,
                      a_skeleton->GetUserData() ? a_skeleton->GetUserData()->GetBaseObject()->formID : 0x0, formId);

        //
        Events::SkinSingleHeadGeometryEvent e;
        e.headNode = a_this;
        e.skeleton = a_skeleton;
        e.geometry = a_triShape;

        //
        Events::Sources::SkinSingleHeadGeometryEventSource::GetSingleton()->SendEvent(&e);
    }

    auto BSFaceGenNiNodeHooks::SkinAllGeometry__Hook(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton,
                                                     const bool a_unk) -> void
    {
        //
        auto name = "";
        uint32_t formId = 0x0;

        //
        if (a_skeleton->GetUserData() && a_skeleton->GetUserData()->data.objectReference)
        {
            if (const auto bname = skyrim_cast<RE::TESFullName*>(a_skeleton->GetUserData()->data.objectReference))
            {
                name = bname->GetFullName();
            }

            if (const auto bnpc = skyrim_cast<RE::TESNPC*>(a_skeleton->GetUserData()->data.objectReference);
                bnpc && bnpc->faceNPC)
            {
                formId = bnpc->faceNPC->formID;
            }
        }

        //
        logger::debug("SkinAllGeometry {} {}, {}, (formid {:08x} base form {:08x} head template form {:08x})",
                      a_skeleton->name.c_str(), a_skeleton->GetChildren().size(), name,
                      a_skeleton->GetUserData() ? a_skeleton->GetUserData()->formID : 0x0,
                      a_skeleton->GetUserData() ? a_skeleton->GetUserData()->GetBaseObject()->formID : 0x0, formId);

        //
        Events::SkinAllHeadGeometryEvent e;
        e.skeleton = a_skeleton;
        e.headNode = a_this;

        Events::Sources::SkinAllHeadGeometryEventSource::GetSingleton()->SendEvent(&e);

        //
        if (REL::Module::IsAE())
        {
            SkinAllGeometryCalls(a_this, a_skeleton, a_unk);
        }
        else
        {
            SkinAllGeometry(a_this, a_skeleton, a_unk);
        }

        //
        e.hasSkinned = true;

        //
        Events::Sources::SkinAllHeadGeometryEventSource::GetSingleton()->SendEvent(&e);
    }

    auto BSFaceGenNiNodeHooks::SkinAllGeometry(RE::BSFaceGenNiNode* const a_this, RE::NiNode* a_skeleton,
                                               const bool a_unk) -> void
    {
        if (a_skeleton)
        {
            if (const auto& children = a_this->GetChildren(); !children.empty())
            {
                for (const auto& child : children)
                {
                    if (child)
                    {
                        if (const auto triShape = child->AsTriShape())
                        {
                            SkinSingleGeometry__Hook(a_this, a_skeleton, triShape, a_unk);
                        }
                    }
                }
            }
        }
    }

    auto BSFaceGenNiNodeHooks::ApplyBoneLimitFix() -> void
    {
        const REL::Relocation GeometrySkinningBoneFix{REL::VariantID(24330, 24836, 0x37ADD0),
                                                      REL::VariantOffset(0x58, 0x75, 0x58)};

        struct BoneLimitFix : Xbyak::CodeGenerator
        {
            BoneLimitFix(const uintptr_t a_returnAddr)
            {
                Xbyak::Label ret;

                auto clamp_bone_count = [&](const Xbyak::Reg32 reg)
                {
                    mov(reg, ptr[rax + 0x58]); // skinData->boneCount
                    cmp(reg, 8); // compare with limit
                    jle(ret); // jump if <= 8 (keep value)
                    mov(reg, 8); // clamp to 8 if > 8
                };

                const Xbyak::Reg32 boneReg = REL::Module::IsSE() ? esi : ebp;
                clamp_bone_count(boneReg);

                //
                L(ret);
                jmp(ptr[rip]);
                dq(a_returnAddr);
            }
        };

        //
        BoneLimitFix code(GeometrySkinningBoneFix.address() + 7);

        //
        auto& localTrampoline = SKSE::GetTrampoline();
        SKSE::AllocTrampoline(14 + code.getSize());

        //
        localTrampoline.write_branch<5>(GeometrySkinningBoneFix.address(), localTrampoline.allocate(code));
    }

    auto MainHooks::Update(RE::Main* const a_this) -> void
    {
        //
        _Update(a_this);

        // why doesn't this class have a GetRuntimeData() helper? the offsets are borked with VR support enabled.

        if (const bool quitGame = REL::RelocateMember<bool>(a_this, 0x10, 0x8))
        {
            const Events::ShutdownEvent e;
            Events::Sources::ShutdownEventEventSource::GetSingleton()->SendEvent(&e);
        }
        else
        {
            Events::FrameEvent e;
            e.gamePaused = a_this->freezeTime;
            Events::Sources::FrameEventSource::GetSingleton()->SendEvent(&e);
        }
    }

    auto MainHooks::Unk_sub(std::any a_this) -> void
    {
        _Unk_sub(a_this);

        //
        static constexpr Events::FrameSyncEvent framesyncEvent;
        Events::Sources::FrameSyncEventSource::GetSingleton()->SendEvent(std::addressof(framesyncEvent));
    }

    auto ActorEquipManagerHooks::func(RE::ActorEquipManager* const a_this, RE::Actor* a_actor,
                                      RE::TESBoundObject* a_object, RE::ExtraDataList* a_extraData,
                                      const std::uint32_t a_count, const RE::BGSEquipSlot* a_slot,
                                      const bool a_queueEquip, const bool a_forceEquip, const bool a_playSounds,
                                      const bool a_applyNow, const RE::BGSEquipSlot* a_slotToReplace) -> bool
    {
        Events::ArmorDetachEvent event;
        event.actor = a_actor;
        event.hasDetached = false;

        //

        Events::Sources::ArmorDetachEventSource::GetSingleton()->SendEvent(&event);

        //
        const auto ret = _func(a_this, a_actor, a_object, a_extraData, a_count, a_slot, a_queueEquip, a_forceEquip,
                               a_playSounds, a_applyNow, a_slotToReplace);

        //
        event.hasDetached = true;

        //
        Events::Sources::ArmorDetachEventSource::GetSingleton()->SendEvent(&event);

        //
        return ret;
    }

    auto BipedAnimHooks::func(RE::BipedAnim* const a_this, RE::NiNode* armor, RE::BSFadeNode* skeleton,
                              const uint32_t a_unk1, void* a_unk2, void* a_unk3, void* a_unk4) -> RE::NiAVObject*
    {
        Events::ArmorAttachEvent armorAtachEvent;

        //
        armorAtachEvent.armorModel = armor;
        armorAtachEvent.skeleton = skeleton;
        armorAtachEvent.attachedNode = nullptr;
        armorAtachEvent.hasAttached = false;

        //
        Events::Sources::ArmorAttachEventSource::GetSingleton()->SendEvent(&armorAtachEvent);

        // tmp fix until we figure out why _func is failing on some stuff.
        std::unordered_map<std::string, std::vector<RE::NiPointer<RE::NiAVObject>>> backupBones;

        //
        if (armor)
        {
            for (auto& NodeName : BackupNodes)
            {
                std::vector<RE::NiPointer<RE::NiAVObject>> result;

                //
                RE::NiAVObject* object = armor->GetObjectByName(NodeName);
                if (RE::BSTriShape* triShape = object ? object->AsTriShape() : nullptr)
                {
                    const auto size = triShape->GetGeometryRuntimeData().skinInstance->skinData->bones;
                    for (auto idx = 0; std::cmp_less(idx, size); idx++) // all good here
                    {
                        const auto bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx];
                        result.emplace_back(hdt::make_nismart(bone));
                    }
                }

                if (!result.empty())
                {
                    backupBones.insert({NodeName, result});
                }
            }
        }

        //
        RE::NiAVObject* ret = _func(a_this, armor, skeleton, a_unk1, a_unk2, a_unk3, a_unk4);

        //
        if (ret)
        {
            for (auto& NodeName : BackupNodes)
            {
                RE::NiAVObject* object = ret->GetObjectByName(NodeName);
                if (RE::BSTriShape* triShape = object ? object->AsTriShape() : nullptr)
                {
                    const auto size = triShape->GetGeometryRuntimeData().skinInstance->skinData->bones;
                    for (auto idx = 0; std::cmp_less(idx, size); idx++)
                    {
                        auto bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx];
                        if (bone == nullptr)
                        {
                            if (backupBones.contains(NodeName))
                            {
                                bone = triShape->GetGeometryRuntimeData().skinInstance->bones[idx] =
                                    backupBones[NodeName][idx].get();
                            }
                        }
                    }
                }
            }
        }

        //
        if (ret)
        {
            armorAtachEvent.attachedNode = ret;
            armorAtachEvent.hasAttached = true;

            //
            Events::Sources::ArmorAttachEventSource::GetSingleton()->SendEvent(&armorAtachEvent);
        }

        //
        return ret;
    }

    auto BSFaceGenNiNodeHooks::SetBoneName_Hook(RE::BSFaceGenModelExtraData* a_fmd, const std::uint32_t a_boneIdx,
                                                RE::BSFixedString* a_boneName) -> void
    {
        // FMD.bones[] has exactly 8 slots (indices 0-7); any index >= 8 is out-of-bounds
        if (a_boneIdx < 8)
        {
            _SetBoneName(a_fmd, a_boneIdx, a_boneName);
        }
    }

    auto BSFaceGenNiNodeHooks::HookSetBoneName() -> void
    {
        static REL::Relocation addr{REL::RelocationID(26303, 26886)};
        _SetBoneName = reinterpret_cast<SetBoneName_t*>(addr.address());
        DetourAttach(reinterpret_cast<PVOID*>(std::addressof(_SetBoneName)), reinterpret_cast<PVOID>(SetBoneName_Hook));
    }

    auto Install() -> void
    {
        logger::trace("Hooking...");

        // generic hooks
        BSFaceGenNiNodeHooks::Hook();
        MainHooks::Hook();

        //
        DetourTransactionBegin();
        DetourUpdateThread(GetCurrentThread());
        ActorEquipManagerHooks::Hook();
        BSFaceGenNiNodeHooks::HookSetBoneName();
        DetourTransactionCommit();

        //
        DetourTransactionBegin();
        DetourUpdateThread(GetCurrentThread());
        BipedAnimHooks::Hook();
        DetourTransactionCommit();

        //
        logger::trace("...success");
    }
} // namespace Hooks
