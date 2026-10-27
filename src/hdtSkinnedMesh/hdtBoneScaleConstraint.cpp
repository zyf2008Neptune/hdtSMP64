#include "hdtBoneScaleConstraint.h"

namespace hdt
{
    BoneScaleConstraint::BoneScaleConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, btTypedConstraint* constraint) :
        m_boneA{a}, m_boneB{b}, m_constraint{constraint}, m_scaleA{1.0f}, m_scaleB{1.0f}
    {}
} // namespace hdt
