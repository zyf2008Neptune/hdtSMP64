#pragma once

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include "hdtSkinnedMeshSystem.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
    class SkinnedMeshWorld : protected btDiscreteDynamicsWorldMt
    {
    public:
        SkinnedMeshWorld();
        ~SkinnedMeshWorld();

        virtual auto addSkinnedMeshSystem(SkinnedMeshSystem* system) -> void;
        virtual auto removeSkinnedMeshSystem(SkinnedMeshSystem* system) -> void;

        auto stepSimulation(btScalar remainingTimeStep, int maxSubSteps = 1,
                            btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) -> int override;

        auto getWind() -> btVector3& { return m_windSpeed; }
        auto getWind() const -> const btVector3& { return m_windSpeed; }

    protected:
        std::vector<float> m_timeSteps;

        virtual auto resetTransformsToOriginal() -> void
        {
            for (int i = 0; i < m_systems.size(); ++i)
            {
                m_systems[i]->resetTransformsToOriginal();
            }
        }

        auto readTransform(float timeStep) -> void
        {
            const size_t n = m_systems.size();
            if (n == 0)
            {
                return;
            }

            m_timeSteps.resize(n);

            // processSkeletonRoot must be ran synchronously to avoid race issues
            for (size_t i = 0; i < n; ++i)
            {
                m_timeSteps[i] = m_systems[i]->prepareForRead(timeStep);
            }

            concurrency::parallel_for(size_t{0}, n, [this](size_t i) { m_systems[i]->readTransform(m_timeSteps[i]); });
        }

        auto writeTransform() -> void
        {
            for (int i = 0; i < m_systems.size(); ++i)
            {
                m_systems[i]->writeTransform();
            }
        }

        auto applyGravity() -> void override;
        auto applyWind() -> void;

        auto predictUnconstraintMotion(btScalar timeStep) -> void override;
        auto integrateTransforms(btScalar timeStep) -> void override;
        auto performDiscreteCollisionDetection() -> void override;
        auto calculateSimulationIslands() -> void override;
        auto solveConstraints(btContactSolverInfo& solverInfo) -> void override;

        std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>> m_systems;

        btVector3 m_windSpeed; // world windspeed

    private:
        std::vector<SkinnedMeshBody*> _bodies;
        std::vector<SkinnedMeshShape*> _shapes;
    };
} // namespace hdt
