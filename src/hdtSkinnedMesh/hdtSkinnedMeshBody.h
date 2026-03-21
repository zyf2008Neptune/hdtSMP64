#pragma once

#include <algorithm>
#include <unordered_set>
#include <vector>

#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <RE/B/BSFixedString.h>
#include <RE/B/BSIntrusiveRefCounted.h>
#include <RE/B/BSTSmartPointer.h>

#include "hdtAABB.h"
#include "hdtBone.h"
#include "hdtBulletHelper.h"
#include "hdtSkinnedMeshBone.h"
#include "hdtVertex.h"

namespace hdt
{
    class SkinnedMeshShape;
#ifdef CUDA
    class CudaBody;
#endif

    class SkinnedMeshBody : public btCollisionObject, public RE::BSIntrusiveRefCounted
    {
    public:
        using btCollisionObject::operator new;
        using btCollisionObject::operator delete;
        using btCollisionObject::operator new[];
        using btCollisionObject::operator delete[];

    public:
        SkinnedMeshBody();
        ~SkinnedMeshBody() override = default;

        struct CollisionShape : public btCollisionShape // a shape only used for markout
        {
            CollisionShape() :
                m_aabb(_mm_setzero_ps(), _mm_setzero_ps()) { m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE; }

            Aabb m_aabb;

            auto getAabb([[maybe_unused]] const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
                -> void override
            {
                aabbMin = m_aabb.m_min;
                aabbMax = m_aabb.m_max;
            }

            auto setLocalScaling([[maybe_unused]] const btVector3& scaling) -> void override {}

            auto getLocalScaling() const -> const btVector3& override
            {
                static const btVector3 ret(1, 1, 1);
                return ret;
            }

            auto calculateLocalInertia([[maybe_unused]] btScalar mass, [[maybe_unused]] btVector3& inertia) const
                -> void override
            {}

            auto getName() const -> const char* override { return "btSkinnedMeshBody"; }
            auto getMargin() const -> btScalar override { return 0; }

            auto setMargin([[maybe_unused]] btScalar m) -> void override {}
        } m_bulletShape;

        struct SkinnedBone
        {
            btMatrix4x3T vertexToBone;
            BoundingSphere localBoundingSphere;
            BoundingSphere worldBoundingSphere;
            SkinnedMeshBone* ptr = nullptr;
            float weightThreshold;
            bool isKinematic;
        };

        RE::BSFixedString m_name;

        //		int m_priority;
        bool m_isKinematic;
        bool m_useBoundingSphere;
        RE::BSTSmartPointer<SkinnedMeshShape> m_shape;

        auto addBone(SkinnedMeshBone* bone, const btQsTransform& verticesToBone, const BoundingSphere& boundingSphere)
            -> int;

        auto finishBuild() -> void;
#ifdef CUDA
        void updateBones();
#endif
        virtual auto internalUpdate() -> void;

        std::vector<SkinnedBone> m_skinnedBones;
#ifdef CUDA
        std::shared_ptr<Bone[]> m_bones;
#else
        std::vector<Bone> m_bones;
#endif

        std::vector<Vertex> m_vertices;
#ifdef CUDA
        std::shared_ptr<VertexPos[]> m_vpos;
#else
        std::vector<VertexPos> m_vpos;
#endif

        std::vector<RE::BSFixedString> m_tags;
        std::unordered_set<RE::BSFixedString> m_canCollideWithTags;
        std::unordered_set<RE::BSFixedString> m_noCollideWithTags;
        std::vector<SkinnedMeshBone*> m_canCollideWithBones;
        std::vector<SkinnedMeshBone*> m_noCollideWithBones;

#ifdef CUDA
        std::shared_ptr<CudaBody> m_cudaObject;
#endif

        auto flexible(const Vertex& v) const -> float;

        auto canCollideWith(const SkinnedMeshBone* bone) const -> bool
        {
            if (!m_canCollideWithBones.empty())
            {
                return std::ranges::find(m_canCollideWithBones, bone) != m_canCollideWithBones.end();
            }
            return std::ranges::find(m_noCollideWithBones, bone) == m_noCollideWithBones.end();
        }

        virtual auto canCollideWith(const SkinnedMeshBody* body) const -> bool;

        auto updateBoundingSphereAabb() -> void;
        auto isBoundingSphereCollided(const SkinnedMeshBody* rhs) const -> bool;
    };
} // namespace hdt
