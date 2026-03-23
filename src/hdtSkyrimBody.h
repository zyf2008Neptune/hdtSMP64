#pragma once

#include <RE/B/BSFixedString.h>

#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"

namespace hdt
{
    class SkyrimSystem;

    struct SkyrimBody : SkinnedMeshBody
    {
        ///* SkinnedMeshBody brings Bullet's operator new/delete into scope (lines 33-36 of hdtSkinnedMeshBody.h), but
        // these using-declarations are not redeclared in SkyrimBody. This leaves delete ambiguous at the SkyrimBody
        // scope under MSVC, causing RE::BSTSmartPointer<SkyrimBody> to fail when Release() invokes the destructor.*///

        using SkinnedMeshBody::operator new;
        using SkinnedMeshBody::operator delete;
        using SkinnedMeshBody::operator new[];
        using SkinnedMeshBody::operator delete[];

        SkyrimBody();
        ~SkyrimBody() override;

        enum class SharedType : uint8_t
        {
            SHARED_PUBLIC,
            SHARED_INTERNAL,
            SHARED_EXTERNAL,
            SHARED_PRIVATE,
        };

        SkyrimSystem* m_mesh;
        SharedType m_shared;
        bool m_disabled = false;
        int m_disablePriority = 0;
        RE::BSFixedString m_disableTag;

        auto canCollideWith(const SkinnedMeshBody* rhs) const -> bool override;
        auto internalUpdate() -> void override;
    };
} // namespace hdt
