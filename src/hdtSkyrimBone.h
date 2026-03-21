#pragma once

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <RE/N/NiNode.h>

#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

namespace hdt
{
    class SkyrimBone : public SkinnedMeshBone
    {
    public:
        SkyrimBone(const RE::BSFixedString& name, RE::NiNode* node, RE::NiNode* skeleton,
                   btRigidBody::btRigidBodyConstructionInfo& ci);

        auto resetTransformToOriginal() -> void override;
        auto readTransform(float timeStep) -> void override;
        auto writeTransform() -> void override;

        int m_depth;
        RE::NiNode* m_node;
        RE::NiNode* m_skeleton;

    private:
        int m_forceUpdateType;
    };
} // namespace hdt
