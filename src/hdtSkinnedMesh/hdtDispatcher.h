#pragma once

#include <ppl.h>
#include <ppltasks.h>
#include <vector>
#include "hdtBulletHelper.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"

namespace hdt
{
    class SkinnedMeshBody;

    class CollisionDispatcher : public btCollisionDispatcherMt
    {
    public:
        CollisionDispatcher(btCollisionConfiguration* collisionConfiguration) :
            btCollisionDispatcherMt(
                collisionConfiguration)
        {
        }

        auto getNewManifold(const btCollisionObject* b0, const btCollisionObject* b1) -> btPersistentManifold* override
        {
            std::scoped_lock l(m_lock);
            auto ret = btCollisionDispatcherMt::getNewManifold(b0, b1);
            return ret;
        }

        auto releaseManifold(btPersistentManifold* manifold) -> void override
        {
            std::scoped_lock l(m_lock);
            btCollisionDispatcherMt::releaseManifold(manifold);
        }

        auto needsCollision(const btCollisionObject* body0, const btCollisionObject* body1) -> bool override;
        auto dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo,
                                       btDispatcher* dispatcher) -> void override;

        auto getNumManifolds() const -> int override;
        auto getInternalManifoldPointer() -> btPersistentManifold** override;
        auto getManifoldByIndexInternal(int index) -> btPersistentManifold* override;

        auto clearAllManifold() -> void;

        SpinLock m_lock;
        std::vector<std::pair<SkinnedMeshBody*, SkinnedMeshBody*>> m_pairs;
    };
}
