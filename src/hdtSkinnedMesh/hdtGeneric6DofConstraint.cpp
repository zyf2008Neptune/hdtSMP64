#include "hdtGeneric6DofConstraint.h"

namespace hdt
{
    Generic6DofConstraint::Generic6DofConstraint(SkinnedMeshBone* a, SkinnedMeshBone* b, const btTransform& frameInA,
                                                 const btTransform& frameInB) :
        BoneScaleConstraint(a, b, static_cast<btTypedConstraint*>(this)),
        btGeneric6DofSpring2Constraint(a->m_rig, b->m_rig, btTransform::getIdentity(), btTransform::getIdentity(),
                                       RO_XYZ)
    {
        const auto fa = a->m_rigToLocal * frameInA;
        const auto fb = b->m_rigToLocal * frameInB;
        setFrames(fa, fb);

        for (int i = 0; i < 6; ++i)
        {
            enableSpring(i, true);
        }

        // m_linearLimits.m_stopERP.setValue(0.1f, 0.1f, 0.1f);

        // m_oldLinearDiff.setZero();
        // m_oldAngularDiff.setZero();
        // m_linearBounce.setZero();
    }

    auto Generic6DofConstraint::scaleConstraint() -> void
    {
        const auto newScaleA = m_boneA->m_currentTransform.getScale();
        const auto newScaleB = m_boneB->m_currentTransform.getScale();

        if (btFuzzyZero(newScaleA - m_scaleA) && btFuzzyZero(newScaleB - m_scaleB))
        {
            return;
        }

        const auto w0 = m_boneA->m_rig.getInvMass();
        const auto w1 = m_boneB->m_rig.getInvMass();
        const auto factorA = newScaleA / m_scaleA;
        const auto factorB = newScaleB / m_scaleB;
        const auto factor = (factorA * w0 + factorB * w1) / (w0 + w1);
        const auto factor2 = factor * factor;
        const auto factor3 = factor2 * factor;
        const auto factor5 = factor3 * factor2;

        getFrameOffsetA().setOrigin(getFrameOffsetA().getOrigin() * factorA);
        getFrameOffsetB().setOrigin(getFrameOffsetB().getOrigin() * factorB);
        m_linearLimits.m_equilibriumPoint *= factor;
        // target k = ma / x(kg/s^2)
        m_linearLimits.m_springStiffness *= factor3;
        m_linearLimits.m_upperLimit *= factor;
        m_linearLimits.m_lowerLimit *= factor;
        for (auto& m_angularLimit : m_angularLimits)
        {
            m_angularLimit.m_springStiffness *= factor5;
        }

        m_scaleA = newScaleA;
        m_scaleB = newScaleB;
    }
} // namespace hdt
