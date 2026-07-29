#include "hdtBoneScaleConstraint.h"

namespace hdt
{
    BoneScaleConstraint::BoneScaleConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, btTypedConstraint* constraint) :
        m_scaleA(1.0F), m_scaleB(1.0F), m_boneA(a), m_boneB(b), m_constraint(constraint)
    {}
} // namespace hdt
