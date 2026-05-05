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
        ~SkinnedMeshWorld() override;

        virtual auto addSkinnedMeshSystem(SkinnedMeshSystem* system) -> void;
        virtual auto removeSkinnedMeshSystem(SkinnedMeshSystem* system) -> void;

        virtual auto updateConstraintsForBone(SkinnedMeshBone* bone) -> void;

        auto stepSimulation(btScalar remainingTimeStep, int maxSubSteps = 1,
                            btScalar fixedTimeStep = static_cast<btScalar>(1.) / static_cast<btScalar>(60.))
            -> int override;

        auto getWind() -> btVector3& { return m_windSpeed; }
        auto getWind() const -> const btVector3& { return m_windSpeed; }

    protected:
        std::vector<float> m_timeSteps;

        auto readTransform(const float timeStep) -> void
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

            tbb::parallel_for(size_t{0}, n, [this](const size_t i) { m_systems[i]->readTransform(m_timeSteps[i]); });
        }

        auto writeTransform() const -> void
        {
            for (const auto& m_system : m_systems)
            {
                m_system->writeTransform();
            }
        }

        auto applyGravity() -> void override;
        auto applyWind() const -> void;

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
