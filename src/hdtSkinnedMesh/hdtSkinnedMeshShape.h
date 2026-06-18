#pragma once
#pragma warning(push)
#pragma warning(disable : 4373)

#include "hdtCollider.h"
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
        virtual ~SkinnedMeshShape() = default;

        virtual auto asPerVertexShape() -> PerVertexShape* { return nullptr; }
        virtual auto asPerTriangleShape() -> PerTriangleShape* { return nullptr; }

        auto getAabb() const -> const Aabb& { return m_tree.aabbAll; }

        virtual auto clipColliders() -> void;
        virtual auto finishBuild() -> void = 0;
        virtual auto internalUpdate() -> void = 0;
        virtual auto markUsedVertices(bool* flags) -> void = 0;
        virtual auto markUsedVertices(std::vector<bool>& flags) -> void = 0;
        virtual auto remapVertices(UINT* map) -> void = 0;

        virtual auto getColliderBoneWeight(const Collider* c, const int boneIdx) -> float = 0;
        virtual auto getColliderBoneIndex(const Collider* c, const int boneIdx) -> int = 0;
        virtual auto baryCoord(const Collider* c, const btVector3& p) -> btVector3 = 0;
        virtual auto baryWeight(const btVector3& w, const int boneIdx) -> float = 0;
        virtual auto getBonePerCollider() -> int = 0;

        SkinnedMeshBody* m_owner;
        vectorA16<Aabb> m_aabb;
        vectorA16<Collider> m_colliders;
        ColliderTree m_tree;
    };

    class PerVertexShape : public SkinnedMeshShape
    {
    public:
        PerVertexShape(SkinnedMeshBody* body);
        ~PerVertexShape() override = default;

        auto asPerVertexShape() -> PerVertexShape* override { return this; }
        auto internalUpdate() -> void override;

        auto getBonePerCollider() -> int final { return 4; }

        auto getColliderBoneWeight(const Collider* c, const int boneIdx) -> float final
        {
            return m_owner->m_vertices[c->vertex].m_weight[boneIdx];
        }

        auto getColliderBoneIndex(const Collider* c, const int boneIdx) -> int final
        {
            return m_owner->m_vertices[c->vertex].getBoneIdx(boneIdx);
        }

        auto baryCoord([[maybe_unused]] const Collider* c, [[maybe_unused]] const btVector3& p) -> btVector3 final
        {
            return {1, 1, 1};
        }

        auto baryWeight([[maybe_unused]] const btVector3& w, [[maybe_unused]] int boneIdx) -> float final { return 1; }

        auto finishBuild() -> void override;
        auto markUsedVertices(bool* flags) -> void override;
        auto markUsedVertices(std::vector<bool>& flags) -> void override;
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
        ~PerTriangleShape() override = default;

        auto asPerVertexShape() -> PerVertexShape* override { return m_verticesCollision.get(); }
        auto asPerTriangleShape() -> PerTriangleShape* override { return this; }
        auto internalUpdate() -> void override;

        auto getBonePerCollider() -> int final { return 12; }

        auto getColliderBoneWeight(const Collider* c, const int boneIdx) -> float final
        {
            return m_owner->m_vertices[c->vertices[boneIdx / 4]].m_weight[boneIdx % 4];
        }

        auto getColliderBoneIndex(const Collider* c, const int boneIdx) -> int final
        {
            return m_owner->m_vertices[c->vertices[boneIdx / 4]].getBoneIdx(boneIdx % 4);
        }

        auto baryCoord(const Collider* c, const btVector3& p) -> btVector3 final
        {
            auto point0 = m_owner->m_vpos[c->vertices[0]].pos();
            auto point1 = m_owner->m_vpos[c->vertices[1]].pos();
            auto point2 = m_owner->m_vpos[c->vertices[2]].pos();
            auto side0 = point0 - p;
            auto side1 = point1 - p;
            auto side2 = point2 - p;
            auto area0 = btCross(side0, side1).get128();
            auto area1 = btCross(side1, side2).get128();
            auto area2 = btCross(side2, side0).get128();
            area0 = _mm_dp_ps(area0, area0, 0x74);
            area1 = _mm_dp_ps(area1, area1, 0x71);
            area2 = _mm_dp_ps(area2, area2, 0x72);
            area0 = _mm_or_ps(area0, area1);
            area0 = _mm_or_ps(area0, area2);
            area0 = _mm_sqrt_ps(area0);
            area1 = _mm_set_ps1(1);
            area1 = _mm_dp_ps(area1, area0, 0x77);
            return _mm_div_ps(area0, area1);
        }

        auto baryWeight(const btVector3& w, const int boneIdx) -> float final { return w[boneIdx / 4]; }

        auto finishBuild() -> void override;
        auto markUsedVertices(bool* flags) -> void override;
        auto markUsedVertices(std::vector<bool>& flags) -> void override;
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
#pragma warning(pop)
