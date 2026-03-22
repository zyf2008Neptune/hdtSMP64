#pragma once

#include <LinearMath/btScalar.h>

#include "hdtCollisionAlgorithm.h"
#include "hdtCollider.h"
#include "hdtSkinnedMeshBody.h"

namespace hdt
{
    class PerVertexShape;
    class PerTriangleShape;
#ifdef CUDA
    class CudaPerVertexShape;
    class CudaPerTriangleShape;
#endif // CUDA

    class SkinnedMeshShape :
        public RE::BSIntrusiveRefCounted
    {
    public:
        BT_DECLARE_ALIGNED_ALLOCATOR()

        SkinnedMeshShape(SkinnedMeshBody* body);
        virtual ~SkinnedMeshShape() {}

        virtual auto asPerVertexShape() -> PerVertexShape* { return nullptr; }
        virtual auto asPerTriangleShape() -> PerTriangleShape* { return nullptr; }

        auto getAabb() const -> const Aabb& { return m_tree.aabbAll; }

        virtual auto clipColliders() -> void;
        virtual auto finishBuild() -> void = 0;
        virtual auto internalUpdate() -> void = 0;
        virtual auto getBonePerCollider() -> int = 0;
        virtual auto markUsedVertices(bool* flags) -> void = 0;
        virtual auto remapVertices(UINT* map) -> void = 0;

        virtual auto getColliderBoneWeight(const Collider* c, int boneIdx) -> float = 0;
        virtual auto getColliderBoneIndex(const Collider* c, int boneIdx) -> int = 0;
#ifndef CUDA
        virtual auto baryCoord(const Collider* c, const btVector3& p) -> btVector3 = 0;
        virtual auto baryWeight(const btVector3& w, int boneIdx) -> float = 0;
#endif // !CUDA

        SkinnedMeshBody* m_owner;
#ifdef CUDA
        std::shared_ptr<Aabb[]> m_aabb;
#else
        vectorA16<Aabb> m_aabb;
#endif // CUDA
        vectorA16<Collider> m_colliders;
        ColliderTree m_tree;
        float m_windEffect = 0.f; //effect from xml m_windEffect

#ifdef ENABLE_CL
        cl::Buffer m_aabbCL;
        cl::Buffer m_colliderCL;
        cl::Event m_eDoneCL;
        virtual void internalUpdateCL() = 0;
#endif
    };

    class PerVertexShape : public SkinnedMeshShape
    {
    public:
#ifdef CUDA
        using CudaType = CudaPerVertexShape;
#endif // CUDA

        PerVertexShape(SkinnedMeshBody* body);
        ~PerVertexShape() override;

        auto asPerVertexShape() -> PerVertexShape* override { return this; }

        auto internalUpdate() -> void override;
        auto getBonePerCollider() -> int override { return 4; }

        auto getColliderBoneWeight(const Collider* c, int boneIdx) -> float override
        {
            return m_owner->m_vertices[c->vertex].m_weight[boneIdx];
        }

        auto getColliderBoneIndex(const Collider* c, int boneIdx) -> int override
        {
            return m_owner->m_vertices[c->vertex].getBoneIdx(boneIdx);
        }

#ifndef CUDA
        auto baryCoord([[maybe_unused]] const Collider* c, [[maybe_unused]] const btVector3& p) -> btVector3 override
        {
            return btVector3(1, 1, 1);
        }

        auto baryWeight([[maybe_unused]] const btVector3& w, [[maybe_unused]] int boneIdx) -> float override
        {
            return 1;
        }
#endif // !CUDA
        auto finishBuild() -> void override;
        auto markUsedVertices(bool* flags) -> void override;
        auto remapVertices(UINT* map) -> void override;

        auto autoGen() -> void;

        struct ShapeProp
        {
            float margin = 1.0f;
        } m_shapeProp;

#ifdef CUDA
        std::shared_ptr<CudaPerVertexShape> m_cudaObject;
#endif // CUDA

#ifdef ENABLE_CL
        static hdtCLKernel m_kernel;
        virtual void internalUpdateCL();
#endif
    };

    class PerTriangleShape : public SkinnedMeshShape
    {
    public:
#ifdef CUDA
        using CudaType = CudaPerTriangleShape;
#endif

        PerTriangleShape(SkinnedMeshBody* body);
        ~PerTriangleShape() override;

        auto asPerVertexShape() -> PerVertexShape* override { return m_verticesCollision.get(); }
        auto asPerTriangleShape() -> PerTriangleShape* override { return this; }

        auto internalUpdate() -> void override;
        auto getBonePerCollider() -> int override { return 12; }

        auto getColliderBoneWeight(const Collider* c, int boneIdx) -> float override
        {
            return m_owner->m_vertices[c->vertices[boneIdx / 4]].m_weight[boneIdx % 4];
        }

        auto getColliderBoneIndex(const Collider* c, int boneIdx) -> int override
        {
            return m_owner->m_vertices[c->vertices[boneIdx / 4]].getBoneIdx(boneIdx % 4);
        }

#ifndef CUDA
        auto baryCoord(const Collider* c, const btVector3& p) -> btVector3 override
        {
            return BaryCoord(
                m_owner->m_vpos[c->vertices[0]].pos(),
                m_owner->m_vpos[c->vertices[1]].pos(),
                m_owner->m_vpos[c->vertices[2]].pos(),
                p);
        }

        auto baryWeight(const btVector3& w, int boneIdx) -> float override { return w[boneIdx / 4]; }
#endif
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

#ifdef CUDA
        std::shared_ptr<CudaPerTriangleShape> m_cudaObject;
#endif

#ifdef ENABLE_CL
        static hdtCLKernel m_kernel;
        virtual void internalUpdateCL();
#endif
    };
}
