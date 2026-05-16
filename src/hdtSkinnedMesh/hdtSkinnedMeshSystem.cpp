#include "hdtSkinnedMeshSystem.h"

#include "hdtBoneScaleConstraint.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{

    auto SkinnedMeshSystem::readTransform(const float timeStep) -> void
    {
        if (this->block_resetting)
        {
            return;
        }

        for (const auto& bone : m_bones)
        {
            bone->readTransform(timeStep);
        }

        for (const auto& constraint : m_constraints)
        {
            constraint->scaleConstraint();
        }

        for (const auto& constraintGroup : m_constraintGroups)
        {
            constraintGroup->scaleConstraint();
        }
    }

    auto SkinnedMeshSystem::writeTransform() -> void
    {
        for (const auto& m_bone : m_bones)
        {
            if (m_bone->m_rig.isKinematicObject())
            {
                continue;
            }

            m_bone->writeTransform();
        }
    }

    auto SkinnedMeshSystem::internalUpdate() const -> void
    {
        for (const auto& bone : m_bones)
        {
            bone->internalUpdate();
        }

        for (const auto& mesh : m_meshes)
        {
            mesh->updateBoundingSphereAabb();
        }
    }

    auto SkinnedMeshSystem::gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes) const
        -> void
    {
        for (auto& mesh : m_meshes)
        {
            bodies.push_back(mesh.get());
            shapes.push_back(mesh->m_shape.get());

            if (const auto triShape = static_cast<PerTriangleShape*>(mesh->m_shape.get()))
            {
                shapes.push_back(triShape->m_verticesCollision.get());
            }
        }
    }
} // namespace hdt
