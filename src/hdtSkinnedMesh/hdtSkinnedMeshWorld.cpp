#include "hdtSkinnedMeshWorld.h"

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <utility>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include <LinearMath/btQuickprof.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btThreads.h>
#include <LinearMath/btTransformUtil.h>
#include <LinearMath/btVector3.h>
#include <PCH.h>

#include "hdtBoneScaleConstraint.h"
#include "hdtDispatcher.h"
#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtSkinnedMeshSystem.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
    SkinnedMeshWorld::SkinnedMeshWorld() :
        btDiscreteDynamicsWorldMt(nullptr, nullptr, m_solverPool, &m_constraintSolver, nullptr)
    {
        btSetTaskScheduler(btGetPPLTaskScheduler());

        m_windSpeed = _mm_setzero_ps();

        const auto collisionConfiguration = new btDefaultCollisionConfiguration;
        const auto collisionDispatcher = new CollisionDispatcher(collisionConfiguration);
        SkinnedMeshAlgorithm::registerAlgorithm(collisionDispatcher);
        m_dispatcher1 = collisionDispatcher;

        const auto broadphase = new btDbvtBroadphase();
        m_broadphasePairCache = broadphase;
        m_solverPool = new btConstraintSolverPoolMt(BT_MAX_THREAD_COUNT);

        // m_islandManager->~btSimulationIslandManager();
        // new (m_islandManager) SimulationIslandManager();
    }

    SkinnedMeshWorld::~SkinnedMeshWorld()
    {
        for (const auto& system : m_systems)
        {
            for (const auto& m_meshe : system->m_meshes)
            {
                removeCollisionObject(m_meshe.get());
            }

            for (const auto& m_constraint : system->m_constraints)
            {
                removeConstraint(m_constraint->m_constraint);
            }

            for (const auto& m_bone : system->m_bones)
            {
                removeRigidBody(&m_bone->m_rig);
            }

            for (const auto& i : system->m_constraintGroups)
            {
                for (const auto& j : i->m_constraints)
                {
                    removeConstraint(j->m_constraint);
                }
            }
        }

        m_systems.clear();
    }

    auto SkinnedMeshWorld::addSkinnedMeshSystem(SkinnedMeshSystem* system) -> void
    {
        if (std::find(m_systems.begin(), m_systems.end(), system) != m_systems.end())
        {
            return;
        }

        m_systems.push_back(hdt::make_smart(system));

        for (const auto& m_meshe : system->m_meshes)
        {
            addCollisionObject(m_meshe.get(), 1, 1);
        }

        for (const auto& m_bone : system->m_bones)
        {
            m_bone->m_rig.setActivationState(DISABLE_DEACTIVATION);
            addRigidBody(&m_bone->m_rig, 0, 0);
        }

        for (const auto i : system->m_constraintGroups)
        {
            for (const auto j : i->m_constraints)
            {
                addConstraint(j->m_constraint, true);
            }
        }

        for (const auto& m_constraint : system->m_constraints)
        {
            addConstraint(m_constraint->m_constraint, true);
        }

        // -10 allows RESET_PHYSICS down the calls. But equality with a float?...
        system->readTransform(RESET_PHYSICS);

        system->m_world = this;
    }

    auto SkinnedMeshWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system) -> void
    {
        const auto idx = std::find(m_systems.begin(), m_systems.end(), system);
        if (idx == m_systems.end())
        {
            return;
        }

        for (const auto i : system->m_constraintGroups)
        {
            for (const auto j : i->m_constraints)
            {
                removeConstraint(j->m_constraint);
            }
        }

        for (const auto& m_meshe : system->m_meshes)
        {
            removeCollisionObject(m_meshe.get());
        }
        for (const auto& m_constraint : system->m_constraints)
        {
            removeConstraint(m_constraint->m_constraint);
        }
        for (const auto& m_bone : system->m_bones)
        {
            removeRigidBody(&m_bone->m_rig);
        }

        std::swap(*idx, m_systems.back());
        m_systems.pop_back();

        system->m_world = nullptr;
    }

    auto SkinnedMeshWorld::stepSimulation(btScalar remainingTimeStep, int, btScalar fixedTimeStep) -> int
    {
        applyGravity();
        if (SkyrimPhysicsWorld::get()->m_enableWind)
        {
            applyWind();
        }

        while (remainingTimeStep > fixedTimeStep)
        {
            internalSingleStepSimulation(fixedTimeStep);
            remainingTimeStep -= fixedTimeStep;
        }
        // For the sake of the bullet library, we don't manage a step that would be lower than a 300Hz frame.
        // Review this when (screens / Skyrim) will allow 300Hz+.
        static constexpr auto minPossiblePeriod = 1.0f / 300.0f;
        if (remainingTimeStep > minPossiblePeriod)
        {
            internalSingleStepSimulation(remainingTimeStep);
        }
        clearForces();

        _bodies.clear();
        _shapes.clear();

        return 0;
    }

    auto SkinnedMeshWorld::performDiscreteCollisionDetection() -> void
    {
        for (const auto& m_system : m_systems)
        {
            m_system->internalUpdate();
        }

        btDiscreteDynamicsWorldMt::performDiscreteCollisionDetection();
    }

    auto SkinnedMeshWorld::calculateSimulationIslands() -> void
    {
        BT_PROFILE("calculateSimulationIslands");
        getSimulationIslandManager()->updateActivationState(getCollisionWorld(), getCollisionWorld()->getDispatcher());

        {
            for (int i = 0; i < m_predictiveManifolds.size(); i++)
            {
                const auto manifold = m_predictiveManifolds[i];
                const auto colObj0 = manifold->getBody0();
                const auto colObj1 = manifold->getBody1();
                if (colObj0 && !colObj0->isStaticOrKinematicObject() &&
                    colObj1 && !colObj1->isStaticOrKinematicObject())
                {
                    getSimulationIslandManager()->getUnionFind().unite(colObj0->getIslandTag(),
                                                                       colObj1->getIslandTag());
                }
            }
        }
        {
            const auto numConstraints = m_constraints.size();
            for (int i = 0; i < numConstraints; i++)
            {
                auto constraint = m_constraints[i];
                if (constraint->isEnabled())
                {
                    const auto colObj0 = &constraint->getRigidBodyA();
                    const auto colObj1 = &constraint->getRigidBodyB();

                    if (colObj0 && !colObj0->isStaticOrKinematicObject() &&
                        colObj1 && !colObj1->isStaticOrKinematicObject())
                    {
                        getSimulationIslandManager()->getUnionFind().unite(colObj0->getIslandTag(),
                                                                           colObj1->getIslandTag());
                    }
                }
            }
        }

        // Force all dynamic bodies within a single HDT system into one simulation island.
        // Without this, kinematic bones (added with group=0, mask=0) that anchor constraints
        // between dynamic bones dont merge islands in Bullet's union-find.. This causes
        // dynamic bones from the same system to end up in separate islands, dispatched to
        // different solver threads. Since BT_THREADSAFE is off, getOrInitSolverBody's
        // companionId is not thread-safe, and shared kinematic bodies become a data race one thread writes
        // a companionId indexing its own solver body pool, another thread reads it and indexes into a
        // different pool, dereferencing garbage as a btRigidBody*
        for (const auto& system : m_systems)
        {
            int firstDynamicTag = -1;
            for (const auto& bone : system->m_bones)
            {
                if (!bone->m_rig.isStaticOrKinematicObject())
                {
                    if (firstDynamicTag == -1)
                    {
                        firstDynamicTag = bone->m_rig.getIslandTag();
                    }
                    else
                    {
                        getSimulationIslandManager()->getUnionFind().unite(firstDynamicTag, bone->m_rig.getIslandTag());
                    }
                }
            }
        }

        getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());
    }

    auto SkinnedMeshWorld::applyGravity() -> void
    {
        for (const auto& i : m_systems)
        {
            for (const auto& j : i->m_bones)
            {
                const auto body = std::addressof(j->m_rig);
                if (!body->isStaticOrKinematicObject() && !(body->getFlags() & BT_DISABLE_WORLD_GRAVITY))
                {
                    body->setGravity(m_gravity * j->m_gravityFactor);
                }
            }
        }

        btDiscreteDynamicsWorldMt::applyGravity();
    }

    auto SkinnedMeshWorld::applyWind() const -> void
    {
        for (auto& i : m_systems)
        {
            const auto system = dynamic_cast<SkyrimSystem*>(i.get());
            if (btFuzzyZero(system->m_windFactor)) // skip any systems that aren't affected by wind
            {
                continue;
            }
            for (const auto& j : i->m_bones)
            {
                const auto body = std::addressof(j->m_rig);
                if (!body->isStaticOrKinematicObject() &&
                    rand() % 5) // apply randomly 80% of the time to desync wind across npcs
                {
                    body->applyCentralForce(m_windSpeed * j->m_windFactor * system->m_windFactor);
                }
            }
        }
    }

    auto SkinnedMeshWorld::predictUnconstraintMotion(btScalar timeStep) -> void
    {
        for (auto i = 0; i < m_nonStaticRigidBodies.size(); i++)
        {
            btRigidBody* body = m_nonStaticRigidBodies[i];
            if (!body->isStaticOrKinematicObject())
            {
                // not realistic, just an approximate
                body->applyDamping(timeStep);
                body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
            }
            else
            {
                body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
            }
        }
    }

    auto SkinnedMeshWorld::integrateTransforms(btScalar timeStep) -> void
    {
        for (auto i = 0; i < m_collisionObjects.size(); ++i)
        {
            const auto body = m_collisionObjects[i];
            if (body->isKinematicObject())
            {
                btTransformUtil::integrateTransform(body->getWorldTransform(), body->getInterpolationLinearVelocity(),
                                                    body->getInterpolationAngularVelocity(), timeStep,
                                                    body->getInterpolationWorldTransform());
                body->setWorldTransform(body->getInterpolationWorldTransform());
            }
        }

        const btVector3 limitMin(-1e+9f, -1e+9f, -1e+9f);
        const btVector3 limitMax(1e+9f, 1e+9f, 1e+9f);
        for (auto i = 0; i < m_nonStaticRigidBodies.size(); i++)
        {
            btRigidBody* body = m_nonStaticRigidBodies[i];
            auto lv = body->getLinearVelocity();
            lv.setMax(limitMin);
            lv.setMin(limitMax);
            body->setLinearVelocity(lv);

            auto av = body->getAngularVelocity();
            av.setMax(limitMin);
            av.setMin(limitMax);
            body->setAngularVelocity(av);
        }

        btDiscreteDynamicsWorldMt::integrateTransforms(timeStep);
    }

    auto SkinnedMeshWorld::solveConstraints(btContactSolverInfo& solverInfo) -> void
    {
        BT_PROFILE("solveConstraints");
        if (!m_collisionObjects.size())
        {
            return;
        }

        m_solverPool->prepareSolve(getCollisionWorld()->getNumCollisionObjects(),
                                   getCollisionWorld()->getDispatcher()->getNumManifolds());

        m_constraintSolver.m_groups.clear();
        for (const auto& i : m_systems)
        {
            for (auto& j : i->m_constraintGroups)
            {
                m_constraintSolver.m_groups.emplace_back(j.get());
            }
        }

        btPersistentManifold** manifold = m_dispatcher1->getInternalManifoldPointer();
        const int maxNumManifolds = m_dispatcher1->getNumManifolds();
        m_solverPool->solveGroup(&m_collisionObjects[0], m_collisionObjects.size(), manifold, maxNumManifolds,
                                 &m_constraints[0], m_constraints.size(), solverInfo, m_debugDrawer, m_dispatcher1);

        m_solverPool->allSolved(solverInfo, m_debugDrawer);
        dynamic_cast<CollisionDispatcher*>(m_dispatcher1)->clearAllManifold();
        m_constraintSolver.m_groups.clear();
    }
} // namespace hdt
