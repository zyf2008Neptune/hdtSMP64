#pragma once

#include "hdtCollider.h"
#include "hdtCollisionAlgorithm.h"
#include "hdtSkinnedMeshBody.h"

namespace hdt
{
    class PerVertexShape;
    class PerTriangleShape;

    class SkinnedMeshShape : public RE::BSIntrusiveRefCounted
    {
    public:
        BT_DECLARE_ALIGNED_ALLOCATOR()

        SkinnedMeshShape(SkinnedMeshBody* body);
        virtual ~SkinnedMeshShape();

        virtual auto asPerVertexShape() -> PerVertexShape* { return nullptr; }
        virtual auto asPerTriangleShape() -> PerTriangleShape* { return nullptr; }

        auto getAabb() const -> const Aabb& { return m_tree.aabbAll; }

        virtual auto clipColliders() -> void;
        virtual auto finishBuild() -> void = 0;
        virtual auto internalUpdate() -> void = 0;
        virtual auto markUsedVertices(bool* flags) -> void = 0;
        virtual auto remapVertices(UINT* map) -> void = 0;

        virtual auto getColliderBoneWeight(const Collider* c, int boneIdx) -> float = 0;
        virtual auto getColliderBoneIndex(const Collider* c, int boneIdx) -> int = 0;
        virtual auto baryCoord(const Collider* c, const btVector3& p) -> btVector3 = 0;
        virtual auto baryWeight(const btVector3& w, int boneIdx) -> float = 0;
        virtual auto getBonePerCollider() -> int = 0;

        SkinnedMeshBody* m_owner;
        vectorA16<Aabb> m_aabb;
        vectorA16<Collider> m_colliders;
        ColliderTree m_tree;
        float m_windEffect = 0.f; // effect from xml m_windEffect
    };

    class PerVertexShape : public SkinnedMeshShape
    {
    public:
        PerVertexShape(SkinnedMeshBody* body);
        virtual ~PerVertexShape();

        auto asPerVertexShape() -> PerVertexShape* override { return this; }
        auto internalUpdate() -> void override;

        inline auto getBonePerCollider() -> int override final { return 4; }

        inline auto getColliderBoneWeight(const Collider* c, int boneIdx) -> float override final
        {
            return m_owner->m_vertices[c->vertex].m_weight[boneIdx];
        }

        inline auto getColliderBoneIndex(const Collider* c, int boneIdx) -> int override final
        {
            return m_owner->m_vertices[c->vertex].getBoneIdx(boneIdx);
        }

        inline auto baryCoord([[maybe_unused]] const Collider* c,
                              [[maybe_unused]] const btVector3& p) -> btVector3 override final
        {
            return btVector3(1, 1, 1);
        }

        inline auto baryWeight([[maybe_unused]] const btVector3& w,
                               [[maybe_unused]] int boneIdx) -> float override final
        {
            return 1;
        }

        auto finishBuild() -> void override;
        auto markUsedVertices(bool* flags) -> void override;
        auto remapVertices(UINT* map) -> void override;
        auto autoGen() -> void;

        struct ShapeProp
        {
            float margin = 1.0f;
        } m_shapeProp;
    };

    class PerTriangleShape : public SkinnedMeshShape
    {
    public:
        PerTriangleShape(SkinnedMeshBody* body);
        virtual ~PerTriangleShape();

        auto asPerVertexShape() -> PerVertexShape* override { return m_verticesCollision.get(); }
        auto asPerTriangleShape() -> PerTriangleShape* override { return this; }
        auto internalUpdate() -> void override;

        inline auto getBonePerCollider() -> int override final { return 12; }

        inline auto getColliderBoneWeight(const Collider* c, int boneIdx) -> float override final
        {
            return m_owner->m_vertices[c->vertices[boneIdx / 4]].m_weight[boneIdx % 4];
        }

        inline auto getColliderBoneIndex(const Collider* c, int boneIdx) -> int override final
        {
            return m_owner->m_vertices[c->vertices[boneIdx / 4]].getBoneIdx(boneIdx % 4);
        }

        inline auto baryCoord(const Collider* c, const btVector3& p) -> btVector3 override final
        {
            return BaryCoord(m_owner->m_vpos[c->vertices[0]].pos(), m_owner->m_vpos[c->vertices[1]].pos(),
                             m_owner->m_vpos[c->vertices[2]].pos(), p);
        }

        inline auto baryWeight(const btVector3& w, int boneIdx) -> float override final { return w[boneIdx / 4]; }

        auto finishBuild() -> void override;
        auto markUsedVertices(bool* flags) -> void override;
        auto remapVertices(UINT* map) -> void override;

        auto addTriangle(int p0, int p1, int p2) -> void;

        struct ShapeProp
        {
            float margin = 1.0f;
            float penetration = 1.f;
        } m_shapeProp;

        RE::BSTSmartPointer<PerVertexShape> m_verticesCollision;
    };
} // namespace hdt
