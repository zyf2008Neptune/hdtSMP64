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

        explicit Collider(const uint32_t i0) : vertex{i0} {}

        Collider(const uint32_t i0, const uint32_t i1, const uint32_t i2) : vertices{i0, i1, i2} {}

        Collider(const Collider& rhs) = default;

        auto operator=(const Collider& rhs) -> Collider& = default;

        union
        {
            u32 vertex; // vertexshape
            std::array<u32, 3> vertices; // triangleshape
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

        explicit ColliderTree(const u32 k) : key(k)
        {
            aabbAll.invalidate();
            aabbMe.invalidate();
        }

        Aabb aabbAll;
        Aabb aabbMe;

        u32 isKinematic{};

        Collider* cbuf{nullptr};
        Aabb* aabb{nullptr};
        u32 numCollider{};
        u32 dynCollider{};

        u32 dynChild{};
        vectorA16<ColliderTree> children;

        vectorA16<Collider> colliders;
        u32 key{};

        auto insertCollider(std::span<u32> keys, size_t keyCount, const Collider& c) -> void;
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
