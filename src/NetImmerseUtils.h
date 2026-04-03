#pragma once

#include <cstdint>
#include <fstream>
#include <string>

#include <RE/B/BSDynamicTriShape.h>
#include <RE/B/BSFixedString.h>
#include <RE/B/BSResourceNiBinaryStream.h>
#include <RE/B/BSTriShape.h>
#include <RE/N/NiAVObject.h>
#include <RE/N/NiNode.h>

namespace hdt
{
    // Returns true if obj looks like a valid NiObject safe to call virtuals on.
    // VR's NiStream can leave bones[] slots as null or as raw char* pointers for
    // unresolved bone references. A char* used as a pointer has its first 8 bytes
    // interpreted as a vtable — those bytes are ASCII text, giving a non-canonical
    // address (> 0x7FFFFFFFFFFF) that faults on access. We also guard against null
    // vtable slots (stub objects from unknown NIF block types).
    // NiObject vtable layout: [0]=~NiRefObject, [1]=DeleteThis, [2]=GetRTTI, [3]=AsNode
    inline constexpr auto kCanonicalUserSpaceMax = 0x00007FFFFFFFFFFFull;

    inline auto isValidNiObject(const RE::NiAVObject* obj) -> bool
    {
        if (!obj)
        {
            return false;
        }
        if (reinterpret_cast<uintptr_t>(obj) > kCanonicalUserSpaceMax)
        {
            return false;
        }
        auto vtbl = *reinterpret_cast<void* const* const*>(obj);
        if (!vtbl || reinterpret_cast<uintptr_t>(vtbl) > kCanonicalUserSpaceMax)
        {
            return false;
        }
        return vtbl[3] != nullptr; // slot 3 = AsNode
    }

    // Returns true if obj is a VR NiStream stub: passes isValidNiObject but has a
    // null or non-canonical function pointer at vtable slot 0x2B (GetObjectByName in VR,
    // offset 0x158). VR uses slot 0x2B because ApplyLocalTransformToWorld is inserted at
    // slot 26, shifting all SKYRIM_REL_VR_VIRTUAL entries by +1 relative to SE's 0x2A.
    // VR creates these stubs for SE-specific NIF block types it can't fully instantiate.
    inline auto isVRNiStreamStub(const RE::NiAVObject* obj) -> bool
    {
        if (!isValidNiObject(obj))
        {
            return false;
        }
        auto vtbl = *reinterpret_cast<const uintptr_t* const*>(obj);
        auto slotGetObjectByName = vtbl[0x158 / sizeof(uintptr_t)]; // 0x158 = slot 0x2B (VR)
        return slotGetObjectByName == 0 || slotGetObjectByName > kCanonicalUserSpaceMax;
    }

    inline auto setNiNodeName(RE::NiNode* node, const char* name) -> void
    {
        node->name = name;
    }

    inline auto castNiNode(RE::NiAVObject* obj) -> RE::NiNode*
    {
        return obj ? obj->AsNode() : nullptr;
    }

    inline auto castBSTriShape(RE::NiAVObject* obj) -> RE::BSTriShape*
    {
        return obj ? obj->AsTriShape() : nullptr;
    }

    inline auto castBSDynamicTriShape(RE::NiAVObject* obj) -> RE::BSDynamicTriShape*
    {
        return obj ? obj->AsDynamicTriShape() : nullptr;
    }

    inline auto findObject(RE::NiAVObject* obj, const RE::BSFixedString& name) -> RE::NiAVObject*
    {
        return obj ? obj->GetObjectByName(name) : nullptr;
    }

    inline auto findNode(RE::NiNode* obj, const RE::BSFixedString& name) -> RE::NiNode*
    {
        const auto ret = dynamic_cast<RE::NiNode*>(findObject(obj, name));
        return ret ? ret->AsNode() : nullptr;
    }

    inline auto readAllFile(const char* path) -> std::string
    {
        RE::BSResourceNiBinaryStream stream(path);
        if (!stream.good())
        {
            return {};
        }

        //
        std::string file;

        //
        size_t required = stream.stream->totalSize;

        //
        file.resize(required);

        //
        stream.read(file.data(), static_cast<uint32_t>(required));

        //
        return file;
    }

    inline auto readAllFile2(const char* path) -> std::string
    {
        std::ifstream stream(path, std::ios::binary);
        if (!stream.is_open())
        {
            return {};
        }

        stream.seekg(0, std::ios::end);
        const auto size = stream.tellg();
        stream.seekg(0, std::ios::beg);
        std::string ret;
        ret.resize(size);
        stream.read(ret.data(), size);
        return ret;
    }

    inline auto updateTransformUpDown(RE::NiAVObject* obj, bool dirty) -> void
    {
        if (!obj)
        {
            return;
        }

        RE::NiUpdateData ctx = {
            0.f,
            dirty ? RE::NiUpdateData::Flag::kDirty : RE::NiUpdateData::Flag::kNone
        };

        //
        obj->UpdateWorldData(&ctx);

        //
        if (RE::NiNode* node = castNiNode(obj))
        {
            for (auto& child : node->GetChildren())
            {
                if (child)
                {
                    updateTransformUpDown(child.get(), dirty);
                }
            }
        }
    }
}
