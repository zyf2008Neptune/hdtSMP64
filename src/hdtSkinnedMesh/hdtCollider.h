#pragma once

#include <functional>
#include "hdtAABB.h"

namespace hdt
{
    struct alignas(16) Collider
    {
        Collider() {}

        Collider(int i0) { vertex = i0; }
        Collider(int i0, int i1, int i2) { vertices[0] = i0, vertices[1] = i1, vertices[2] = i2; }
        Collider(const Collider& rhs) { operator=(rhs); }

        auto operator=(const Collider& rhs) -> Collider&
        {
            __m128i* dst = (__m128i*)this;
            __m128i* src = (__m128i*)&rhs;
            auto xmm0 = _mm_load_si128(src + 0);
            _mm_store_si128(dst, xmm0);
            return *this;
        }

        union
        {
            U32 vertex; // vertexshape
            U32 vertices[3]; // triangleshape
        };

        float flexible;
        //		inline bool operator <(const Collider& rhs){ return aligned < rhs.aligned; }
    };

    struct alignas(16) ColliderTree
    {
        ColliderTree()
        {
            aabbAll.invalidate();
            aabbMe.invalidate();
        }

        ColliderTree(U32 k) :
            key(k)
        {
            aabbAll.invalidate();
            aabbMe.invalidate();
        }

        Aabb aabbAll;
        Aabb aabbMe;

        U32 isKinematic;

        Collider* cbuf = nullptr;
        Aabb* aabb;
        U32 numCollider;
        U32 dynCollider;

        U32 dynChild;
        vectorA16<ColliderTree> children;

        vectorA16<Collider> colliders;
        U32 key;

        auto insertCollider(const U32* keys, size_t keyCount, const Collider& c) -> void;
        auto exportColliders(vectorA16<Collider>& exportTo) -> void;
        auto remapColliders(Collider* start, Aabb* startAabb) -> void;

        auto checkCollisionL(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret) -> void;
        auto checkCollisionR(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret) -> void;
        auto clipCollider(const std::function<bool(const Collider&)>& func) -> void;
        auto updateKinematic(const std::function<float(const Collider*)>& func) -> void;
        auto visitColliders(const std::function<void(Collider*)>& func) -> void;
        auto updateAabb() -> void;
        auto optimize() -> void;

        auto empty() const -> bool { return children.empty() && colliders.empty(); }

        auto collapseCollideL(ColliderTree* r) -> bool;
        auto collapseCollideR(ColliderTree* r) -> bool;
    };
} // namespace hdt
