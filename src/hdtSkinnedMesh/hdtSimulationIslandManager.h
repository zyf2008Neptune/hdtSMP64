#pragma once

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <BulletCollision/BroadphaseCollision/btDispatcher.h>

namespace hdt
{
	class SimulationIslandManager : public btSimulationIslandManager
	{
	public:
		void updateActivationState(btCollisionWorld* colWorld, btDispatcher* dispatcher) override;

		void findUnions(btDispatcher* dispatcher, btCollisionWorld* colWorld);
		//void buildAndProcessIslands(btDispatcher* dispatcher, btCollisionWorld* collisionWorld, IslandCallback* callback);
	};
}
