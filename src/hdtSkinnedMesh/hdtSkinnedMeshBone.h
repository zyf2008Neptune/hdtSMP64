#pragma once

#include <vector>

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <corecrt.h>

#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>
#include <RE/B/BSIntrusiveRefCounted.h>
#include <RE/B/BSFixedString.h>

#include "hdtBulletHelper.h"

namespace hdt
{
    class SkinnedMeshBody;
    _CRT_ALIGN(16)
    struct SkinnedMeshBone :
        public RE::BSIntrusiveRefCounted
    {
        BT_DECLARE_ALIGNED_ALLOCATOR()

        SkinnedMeshBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci);
        virtual ~SkinnedMeshBone() = default;

        RE::BSFixedString m_name;
        float m_marginMultipler;
        float m_boudingSphereMultipler = 1.0f;
        float m_gravityFactor = 1.0f;
        float m_windFactor = 1.0f;
        // wind factor for each skinnedmeshbody; currently not changed, this may have been intended to be m_windEffect e.g., (wind-effect in xml)

        btRigidBody m_rig;
        btTransform m_localToRig;
        btTransform m_rigToLocal;
        btQsTransform m_currentTransform;
        btQsTransform m_origTransform;
        btQsTransform m_origToSkeletonTransform;

        std::vector<RE::BSFixedString> m_canCollideWithBone;
        std::vector<RE::BSFixedString> m_noCollideWithBone;

        virtual auto resetTransformToOriginal() -> void = 0;
        virtual auto readTransform(float timeStep) -> void = 0;
        virtual auto writeTransform() -> void = 0;

        auto internalUpdate() -> void;

        auto canCollideWith(SkinnedMeshBone* rhs) -> bool;
    };
}
