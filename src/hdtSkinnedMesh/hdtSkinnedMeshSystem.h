#pragma once

#include <tbb/tbb.h>

#include "hdtBulletHelper.h"
#include "hdtConstraintGroup.h"

namespace hdt
{
    struct SkinnedMeshBone;
    class SkinnedMeshBody;
    class SkinnedMeshShape;
    class SkinnedMeshWorld;
    class BoneScaleConstraint;

    class SkinnedMeshSystem : public RE::BSIntrusiveRefCounted
    {
        friend class SkinnedMeshWorld;

    public:
        virtual ~SkinnedMeshSystem() = default;

        virtual auto prepareForRead(float timeStep) -> float { return timeStep; }
        virtual auto resetTransformsToOriginal() -> void;
        virtual auto readTransform(float timeStep) -> void;
        virtual auto writeTransform() -> void;

        auto internalUpdate() const -> void;

        auto gather(std::vector<SkinnedMeshBody*>& bodies, std::vector<SkinnedMeshShape*>& shapes) const -> void;

        auto valid() const -> bool { return !m_bones.empty(); }

        std::vector<std::shared_ptr<btCollisionShape>> m_shapeRefs;
        SkinnedMeshWorld* m_world = nullptr;

        bool block_resetting = false;
        auto getBones() -> std::vector<RE::BSTSmartPointer<SkinnedMeshBone>>& { return m_bones; };

    protected:
        std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> m_bones;
        std::vector<RE::BSTSmartPointer<SkinnedMeshBody>> m_meshes;
        std::vector<RE::BSTSmartPointer<BoneScaleConstraint>> m_constraints;
        std::vector<RE::BSTSmartPointer<ConstraintGroup>> m_constraintGroups;

    private:
        using task_group = tbb::task_group;
    };
} // namespace hdt
