#include "hdtSkyrimBody.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
    SkyrimBody::SkyrimBody()
    {
        m_mesh = nullptr;
        m_shared = SharedType::SHARED_PUBLIC;
    }

    SkyrimBody::~SkyrimBody() {}

    auto SkyrimBody::canCollideWith(const SkinnedMeshBody* rhs) const -> bool
    {
        const auto body = dynamic_cast<const SkyrimBody*>(rhs);

        if (!body)
        {
            return SkinnedMeshBody::canCollideWith(rhs);
        }

        if (m_disabled || body->m_disabled)
        {
            return false;
        }

        switch (m_shared)
        {
        case SharedType::SHARED_PUBLIC:
            break;
        case SharedType::SHARED_INTERNAL:
        {
            if (!m_mesh || !body->m_mesh)
            {
                break;
            }
            if (m_mesh->m_skeleton != body->m_mesh->m_skeleton)
            {
                return false;
            }
            break;
        }
        case SharedType::SHARED_EXTERNAL:
        {
            if (!m_mesh || !body->m_mesh)
            {
                break;
            }
            if (m_mesh->m_skeleton == body->m_mesh->m_skeleton)
            {
                return false;
            }
            break;
        }
        case SharedType::SHARED_PRIVATE:
        {
            if (!m_mesh || !body->m_mesh)
            {
                break;
            }
            if (m_mesh != body->m_mesh)
            {
                return false;
            }
            break;
        }
        }

        return SkinnedMeshBody::canCollideWith(rhs);
    }

    auto SkyrimBody::internalUpdate() -> void
    {
        if (m_disabled)
        {
            return;
        }
        SkinnedMeshBody::internalUpdate();
    }
} // namespace hdt
