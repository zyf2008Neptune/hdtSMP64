#include "hdtSkinnedMeshBone.h"

namespace hdt
{
    SkinnedMeshBone::SkinnedMeshBone(const RE::BSFixedString& name,
                                     const btRigidBody::btRigidBodyConstructionInfo& ci) : m_name(name), m_rig(ci)
    {
        m_rigToLocal.setIdentity();
        m_localToRig.setIdentity();
        m_currentTransform.setScale(1);

        m_marginMultipler = 1.0f;

        m_rig.setUserPointer(this);
    }

    auto SkinnedMeshBone::internalUpdate() -> void
    {
        auto t = m_rigToLocal * m_rig.getInterpolationWorldTransform();
        m_currentTransform.setBasis(t.getBasis());
        m_currentTransform.setOrigin(t.getOrigin());
    }

    auto SkinnedMeshBone::canCollideWith(const SkinnedMeshBone* rhs) -> bool
    {
        if (m_canCollideWithBone.size())
        {
            return std::ranges::find(m_canCollideWithBone, rhs->m_name) != m_canCollideWithBone.end();
        }
        return std::ranges::find(m_noCollideWithBone, rhs->m_name) == m_noCollideWithBone.end();
    }
} // namespace hdt
