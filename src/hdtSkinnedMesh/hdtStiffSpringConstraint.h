#pragma once

#include "hdtBoneScaleConstraint.h"
#include "hdtBulletHelper.h"

namespace hdt
{
    class StiffSpringConstraint :
        public BoneScaleConstraint,
        public btTypedConstraint
    {
    public:
        BT_DECLARE_ALIGNED_ALLOCATOR()

        StiffSpringConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b);
        virtual ~StiffSpringConstraint() override = default;

        auto scaleConstraint() -> void override;

        float m_minDistance;
        float m_maxDistance;
        float m_stiffness;
        float m_damping;
        float m_equilibriumPoint;

    protected:
        float m_oldDiff;

        ///internal method used by the constraint solver, don't use them directly
        auto getInfo1(btConstraintInfo1* info) -> void override;
        ///internal method used by the constraint solver, don't use them directly
        auto getInfo2(btConstraintInfo2* info) -> void override;

        ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
        ///If no axis is provided, it uses the default axis for this constraint.
        auto setParam([[maybe_unused]] int num, [[maybe_unused]] btScalar value,
                      [[maybe_unused]] int axis = -1) -> void override {
        }

        ///return the local value of parameter
        auto getParam([[maybe_unused]] int num, [[maybe_unused]] int axis = -1) const -> btScalar override { return 0; }
    };
}
