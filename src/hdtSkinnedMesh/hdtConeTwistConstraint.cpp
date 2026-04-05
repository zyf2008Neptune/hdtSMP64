#include "hdtConeTwistConstraint.h"

namespace hdt
{
    ConeTwistConstraint::ConeTwistConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, const btTransform& frameInA,
                                             const btTransform& frameInB) :
        BoneScaleConstraint(a, b, static_cast<btConeTwistConstraint*>(this)),
        btConeTwistConstraint(a->m_rig, b->m_rig, btTransform::getIdentity(), btTransform::getIdentity())
    {
        const auto fa = a->m_rigToLocal * frameInA;
        const auto fb = b->m_rigToLocal * frameInB;
        btConeTwistConstraint::setFrames(fa, fb);

        enableMotor(false); // motor temporary not support
    }

    auto ConeTwistConstraint::scaleConstraint() -> void
    {
        const auto newScaleA = m_boneA->m_currentTransform.getScale();
        const auto newScaleB = m_boneB->m_currentTransform.getScale();

        if (btFuzzyZero(newScaleA - m_scaleA) && btFuzzyZero(newScaleB - m_scaleB))
        {
            return;
        }

        auto frameA = getFrameOffsetA();
        auto frameB = getFrameOffsetB();
        frameA.setOrigin(frameA.getOrigin() * (newScaleA / m_scaleA));
        frameB.setOrigin(frameB.getOrigin() * (newScaleB / m_scaleB));
        setFrames(frameA, frameB);
        m_scaleA = newScaleA;
        m_scaleB = newScaleB;
    }
}
