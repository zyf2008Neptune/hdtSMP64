#include "hdtSkinnedMeshSystem.h"

#include "hdtBoneScaleConstraint.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
    auto SkinnedMeshSystem::resetTransformsToOriginal() -> void
    {
        for (int i = 0; i < m_bones.size(); ++i)
        {
            m_bones[i]->resetTransformToOriginal();
        }
    }

    auto SkinnedMeshSystem::readTransform(float timeStep) -> void
    {
        if (this->block_resetting)
        {
            return;
        }

        for (const auto& bone : m_bones)
        {
            bone->readTransform(timeStep);
        }

        for (const auto i : m_constraints)
        {
            i->scaleConstraint();
        }

        for (const auto i : m_constraintGroups)
        {
            i->scaleConstraint();
        }
    }

    auto SkinnedMeshSystem::writeTransform() -> void
    {
        for (int i = 0; i < m_bones.size(); ++i)
        {
            if (m_bones[i]->m_rig.isKinematicObject())
            {
                continue;
            }

            m_bones[i]->writeTransform();
        }
    }

    auto SkinnedMeshSystem::internalUpdate() const -> void
    {
        for (const auto& i : m_bones)
        {
            i->internalUpdate();
        }

        for (const auto& i : m_meshes)
        {
            i->updateBoundingSphereAabb();
        }
    }

    auto SkinnedMeshSystem::gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes) const
        -> void
    {
        for (auto& i : m_meshes)
        {
            bodies.push_back(i.get());
            shapes.push_back(i->m_shape.get());

            if (const auto triShape = static_cast<PerTriangleShape*>(i->m_shape.get()))
            {
                shapes.push_back(triShape->m_verticesCollision.get());
            }
        }
    }
} // namespace hdt
