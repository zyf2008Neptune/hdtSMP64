#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include "hdtAABB.h"

namespace hdt
{
    inline constexpr uint32_t MaxCollisionPairs = 6024;

    struct alignas(16) Collider
    {
        Collider() = default;

        Collider(const uint32_t i0) : vertex{i0} {}

        Collider(const uint32_t i0, const uint32_t i1, const uint32_t i2) : vertices{i0, i1, i2} {}

        Collider(const Collider& rhs) { operator=(rhs); }

        auto operator=(const Collider& rhs) -> Collider& = default;

        union
        {
            U32 vertex; // vertexshape
            std::array<U32, 3> vertices; // triangleshape
        };

        float flexible{};
        //		inline bool operator <(const Collider& rhs){ return aligned < rhs.aligned; }
    };

    struct alignas(16) ColliderTree
    {
        ColliderTree()
        {
            aabbAll.invalidate();
            aabbMe.invalidate();
        }

        ColliderTree(const U32 k) : key(k)
        {
            aabbAll.invalidate();
            aabbMe.invalidate();
        }

        Aabb aabbAll;
        Aabb aabbMe;

        U32 isKinematic{};

        Collider* cbuf = nullptr;
        Aabb* aabb = nullptr;
        U32 numCollider{};
        U32 dynCollider{};

        U32 dynChild{};
        vectorA16<ColliderTree> children;

        vectorA16<Collider> colliders;
        U32 key{};

        auto insertCollider(std::span<U32> keys, size_t keyCount, const Collider& c) -> void;
        auto exportColliders(vectorA16<Collider>& exportTo) -> void;
        auto remapColliders(Collider* start, Aabb* startAabb) -> void;

        auto checkCollisionL(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret) -> void;
        auto checkCollisionR(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret) -> void;
        auto clipCollider(const std::function<bool(const Collider&)>& func) -> void;
        auto updateKinematic(const std::function<float(const Collider*)>& func) -> void;
        auto visitColliders(const std::function<void(Collider*)>& func) -> void;
        auto updateAabb() -> void;
        auto optimize() -> void;

        [[nodiscard]] auto empty() const -> bool { return children.empty() && colliders.empty(); }

        auto collapseCollideL(ColliderTree* r) -> bool;
        auto collapseCollideR(ColliderTree* r) -> bool;
    };
} // namespace hdt
