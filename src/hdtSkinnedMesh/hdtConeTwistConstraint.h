#pragma once

#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <LinearMath/btTransform.h>

#include "hdtBoneScaleConstraint.h"
#include "hdtSkinnedMeshBone.h"

namespace hdt
{
    class ConeTwistConstraint :
        public BoneScaleConstraint,
        public btConeTwistConstraint
    {
    public:
        BT_DECLARE_ALIGNED_ALLOCATOR()

        ConeTwistConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, const btTransform& frameInA,
                            const btTransform& frameInB);
        ~ConeTwistConstraint() override = default;

        auto scaleConstraint() -> void override;
    };
}
