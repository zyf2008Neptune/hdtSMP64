#pragma once

#include <cstdint>
#include <new>

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
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

#ifdef CUDA
#include "hdtCudaInterface.h"
// Define this to do actual collision checking on GPU. This is currently slow and has very inconsistent
// framerate. If not defined, the GPU will still be used if available for vertex and bounding box
// calculations, but collision will be done on the CPU.
#define USE_GPU_COLLISION
#endif

namespace hdt
{
    class SkinnedMeshAlgorithm : public btCollisionAlgorithm
    {
    public:
        SkinnedMeshAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);

        auto processCollision([[maybe_unused]] const btCollisionObjectWrapper* body0Wrap,
                              [[maybe_unused]] const btCollisionObjectWrapper* body1Wrap,
                              [[maybe_unused]] const btDispatcherInfo& dispatchInfo,
                              [[maybe_unused]] btManifoldResult* resultOut) -> void override
        {}

        auto calculateTimeOfImpact([[maybe_unused]] btCollisionObject* body0,
                                   [[maybe_unused]] btCollisionObject* body1,
                                   [[maybe_unused]] const btDispatcherInfo& dispatchInfo,
                                   [[maybe_unused]] btManifoldResult* resultOut) -> btScalar override
        {
            return 1;
        } // TOI cost too much
        auto getAllContactManifolds([[maybe_unused]] btManifoldArray& manifoldArray) -> void override {}

        struct CreateFunc : public btCollisionAlgorithmCreateFunc
        {
            auto CreateCollisionAlgorithm(
                btCollisionAlgorithmConstructionInfo& ci, [[maybe_unused]] const btCollisionObjectWrapper* body0Wrap,
                [[maybe_unused]] const btCollisionObjectWrapper* body1Wrap) -> btCollisionAlgorithm* override
            {
                auto mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(SkinnedMeshAlgorithm));
                return new(mem) SkinnedMeshAlgorithm(ci);
            }
        };

        static auto registerAlgorithm(btCollisionDispatcherMt* dispatcher) -> void;

        static constexpr std::uint16_t MaxCollisionCount = 512;

#ifdef CUDA
        static std::function<void()> queueCollision(SkinnedMeshBody* body0Wrap, SkinnedMeshBody* body1Wrap,
                                                    CollisionDispatcher* dispatcher);
#endif

        static auto processCollision(SkinnedMeshBody* body0Wrap, SkinnedMeshBody* body1Wrap,
                                     CollisionDispatcher* dispatcher) -> void;

    protected:
        struct CollisionMerge
        {
            btVector3 normal;
            btVector3 pos[2];
            float weight;

            CollisionMerge()
            {
                _mm_store_ps((reinterpret_cast<float*>(this)), _mm_setzero_ps());
                _mm_store_ps((reinterpret_cast<float*>(this)) + 4, _mm_setzero_ps());
                _mm_store_ps((reinterpret_cast<float*>(this)) + 8, _mm_setzero_ps());
                _mm_store_ps((reinterpret_cast<float*>(this)) + 12, _mm_setzero_ps());
            }
        };

        struct MergeBuffer
        {
            MergeBuffer() = default;

            [[nodiscard]] auto begin() const -> CollisionMerge* { return buffer; }
            [[nodiscard]] auto end() const -> CollisionMerge* { return buffer + mergeSize; }

            auto alloc(const int x, const int y) -> void
            {
                release();
                mergeStride = y;
                mergeSize = x * y;
                buffer = new CollisionMerge[mergeSize];
            }

            auto release() -> void
            {
                delete[] buffer;
                buffer = nullptr;
                mergeStride = 0;
                mergeSize = 0;
            }

            [[nodiscard]] auto get(const int x, const int y) const -> CollisionMerge*
            {
                return std::addressof(buffer[x * mergeStride + y]);
            }

            auto doMerge(SkinnedMeshShape* shape0, SkinnedMeshShape* shape1, CollisionResult* collisions,
                         int count) -> void;

            auto apply(const SkinnedMeshBody* body0, const SkinnedMeshBody* body1,
                       CollisionDispatcher* dispatcher) -> void;

            int mergeStride = 0;
            int mergeSize = 0;
            CollisionMerge* buffer = nullptr;
#ifdef CUDA
            std::mutex lock;
#endif
        };

        template <class T0, class T1>
        static auto processCollision(T0* shape0, T1* shape1, MergeBuffer& merge, CollisionResult* collision) -> void;
    };
} // namespace hdt
