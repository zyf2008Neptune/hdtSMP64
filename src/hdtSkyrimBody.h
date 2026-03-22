#pragma once

#include <RE/B/BSFixedString.h>

#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
    class SkyrimSystem;

    struct SkyrimBody : SkinnedMeshBody
    {
        SkyrimBody();
        ~SkyrimBody();

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
