#pragma once

#include <vector>

#include <BulletDynamics/ConstraintSolver/btContactSolverInfo.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>
#include <RE/B/BSTSmartPointer.h>

#include "hdtGroupConstraintSolver.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshSystem.h"

namespace hdt
{
    class SkinnedMeshWorld : protected btDiscreteDynamicsWorldMt
    {
    public:
        SkinnedMeshWorld();
        ~SkinnedMeshWorld() override;

        virtual auto addSkinnedMeshSystem(SkinnedMeshSystem* system) -> void;
        virtual auto removeSkinnedMeshSystem(SkinnedMeshSystem* system) -> void;

        auto stepSimulation(btScalar remainingTimeStep, int maxSubSteps = 1,
                            btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) -> int override;

        auto getWind() -> btVector3& { return m_windSpeed; }
        auto getWind() const -> const btVector3& { return m_windSpeed; }

    protected:
        virtual auto resetTransformsToOriginal() -> void
        {
            for (const auto& m_system : m_systems)
            {
                m_system->resetTransformsToOriginal();
            }
        }

        auto readTransform(float timeStep) const -> void
        {
            for (const auto& m_system : m_systems)
            {
                m_system->readTransform(timeStep);
            }
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
        auto solveConstraints(btContactSolverInfo& solverInfo) -> void override;

        std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>> m_systems;

        btVector3 m_windSpeed; // world windspeed

    private:
        std::vector<SkinnedMeshBody*> _bodies;
        std::vector<SkinnedMeshShape*> _shapes;
        btConstraintSolverPoolMt* m_solverPool;
        GroupConstraintSolver m_constraintSolver;
    };
}
