#pragma once

#include "hdtConvertNi.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkyrimBone.h"

namespace hdt
{
    class SkyrimSystem;

    class SkyrimBody : public SkinnedMeshBody
    {
    public:
        SkyrimBody();
        ~SkyrimBody() override = default;

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

        auto canCollideWith(const SkinnedMeshBody* body) const -> bool override;
        auto internalUpdate() -> void override;
    };
} // namespace hdt
