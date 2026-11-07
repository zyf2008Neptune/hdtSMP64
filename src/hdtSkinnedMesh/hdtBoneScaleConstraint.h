#pragma once

#include "hdtBulletHelper.h"
#include "hdtSkinnedMeshBody.h"

namespace hdt
{
    class alignas(16) BoneScaleConstraint : public RefObject
    {
    public:
        BoneScaleConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, btTypedConstraint* constraint);
        ~BoneScaleConstraint() override = default;

        virtual auto scaleConstraint() -> void = 0;

        auto getConstraint() const -> btTypedConstraint* { return m_constraint; }

        SkinnedMeshBone* m_boneA{nullptr};
        SkinnedMeshBone* m_boneB{nullptr};
        btTypedConstraint* m_constraint{nullptr};
        float m_scaleA{};
        float m_scaleB{};
    };
} // namespace hdt
