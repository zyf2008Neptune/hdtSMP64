#pragma once

#include <algorithm>
#include <cstdint>

#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>
#include <BulletCollision/BroadphaseCollision/btDispatcher.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>

#include "hdtCollisionAlgorithm.h"
#include "hdtDispatcher.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
    class SkinnedMeshAlgorithm
    {
    public:
        // Note: It's possible to exceed this with complex outfits, which is why we cap it.
        // We don't want to stress a simulation island too much!
        static constexpr int MaxCollisionCount = 512;

        static auto processCollision(const SkinnedMeshBody* body0, const SkinnedMeshBody* body1,
                                     CollisionDispatcher* dispatcher) -> void;

    protected:
        struct CollisionMerge
        {
            btVector3 normal; // accumulated weighted normal: length encodes depth, direction encodes contact normal
            btVector3 pos[2];
            float weight;

            CollisionMerge()
            {
                _mm_store_ps(reinterpret_cast<float*>(this), _mm_setzero_ps());
                _mm_store_ps(reinterpret_cast<float*>(this) + 4, _mm_setzero_ps());
                _mm_store_ps(reinterpret_cast<float*>(this) + 8, _mm_setzero_ps());
                _mm_store_ps(reinterpret_cast<float*>(this) + 12, _mm_setzero_ps());
            }

            auto reset() -> void
            {
                _mm_store_ps(reinterpret_cast<float*>(this), _mm_setzero_ps());
                _mm_store_ps(reinterpret_cast<float*>(this) + 4, _mm_setzero_ps());
                _mm_store_ps(reinterpret_cast<float*>(this) + 8, _mm_setzero_ps());
                _mm_store_ps(reinterpret_cast<float*>(this) + 12, _mm_setzero_ps());
            }
        };

        struct MergeBuffer
        {
            MergeBuffer() :
                mergeStride(0), mergeSize(0), currentGen(0)
            {
                activeCells.reserve(256);
            }

            ~MergeBuffer()
            {
                std::free(buffer);
                std::free(generations);
            }

            MergeBuffer(const MergeBuffer&) = delete;
            auto operator=(const MergeBuffer&) -> MergeBuffer& = delete;

            // Just in case!
            MergeBuffer(MergeBuffer&&) = delete;
            auto operator=(MergeBuffer&&) -> MergeBuffer& = delete;

            // Buffer is ~2-10% occupied in basic scenes, so instead of zeroing old cells every resize() call we just bump a generation counter.
            // cells get lazily reset on first touch in getAndTrack(). Makes resize() O(1).
            auto resize(const int x, const int y) -> void
            {
                mergeStride = y;
                if (const int needed = x * y; needed > mergeSize)
                {
                    std::free(buffer);
                    std::free(generations);
                    mergeSize = needed;
                    buffer = static_cast<CollisionMerge*>(std::malloc(needed * sizeof(CollisionMerge)));
                    generations = static_cast<uint32_t*>(std::calloc(needed, sizeof(uint32_t)));
                }
                if (++currentGen == 0)
                {
                    // wrap around, shouldn't realistically happen (~4 billion frames lol)
                    // This is virtually skipped entirely by the cpu, 0 cost. Just in case since it would
                    // create difficult to track down inconsistencies..
                    std::fill_n(generations, mergeSize * sizeof(uint32_t), 0u);
                    currentGen = 1;
                }
                activeCells.clear();
            }

            auto get(const int x, const int y) const -> CollisionMerge* { return &buffer[x * mergeStride + y]; }

            auto getAndTrack(const int x, const int y) -> CollisionMerge*
            {
                const int idx = x * mergeStride + y;
                auto* c = &buffer[idx];
                if (generations[idx] != currentGen)
                {
                    c->reset();
                    generations[idx] = currentGen;
                    activeCells.push_back(idx);
                }
                return c;
            }

            template <class T0, class T1>
            auto doMerge(T0* shape0, T1* shape1, CollisionResult* collisions, int count) -> void;

            auto apply(const SkinnedMeshBody* body0, const SkinnedMeshBody* body1,
                       CollisionDispatcher* dispatcher) const -> void;

            int mergeStride;
            int mergeSize;
            uint32_t currentGen;
            CollisionMerge* buffer = nullptr;
            uint32_t* generations = nullptr;
            std::vector<int> activeCells;
        };

        template <class T0, class T1>
        static auto processCollision(T0* shape0, T1* shape1, MergeBuffer& merge, CollisionResult* collision) -> void;
    };
} // namespace hdt
