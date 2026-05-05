#pragma once

#include <memory>
#include "hdtAABB.h"
#include "hdtBulletHelper.h"

namespace hdt
{
    class SkinnedMeshBody;
    _CRT_ALIGN(16)
    struct SkinnedMeshBone : RE::BSIntrusiveRefCounted
    {
        BT_DECLARE_ALIGNED_ALLOCATOR()

        SkinnedMeshBone(const RE::BSFixedString& name, const btRigidBody::btRigidBodyConstructionInfo& ci);
        virtual ~SkinnedMeshBone() = default;

        RE::BSFixedString m_name;
        float m_marginMultipler;
        float m_boudingSphereMultipler = 1.0f;
        float m_gravityFactor = 1.0f;
        float m_windFactor = 1.0f; // Mapped to <wind-factor> in the XML. Acts as a multiplier for the global wind force
                                   // applied to this bone (0.0 = no wind, 2.0 = double wind)

        btRigidBody m_rig;
        btTransform m_localToRig;
        btTransform m_rigToLocal;
        btQsTransform m_currentTransform;

        std::vector<RE::BSFixedString> m_canCollideWithBone;
        std::vector<RE::BSFixedString> m_noCollideWithBone;

        virtual auto readTransform(float timeStep) -> void = 0;
        virtual auto writeTransform() -> void = 0;

        auto internalUpdate() -> void;

        auto canCollideWith(const SkinnedMeshBone* rhs) -> bool;
    };
} // namespace hdt
