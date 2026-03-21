#pragma once

#include <memory>
#include <vector>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
#include <BulletDynamics/ConstraintSolver/btContactSolverInfo.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>
#include <BulletDynamics/ConstraintSolver/btSolverBody.h>
#include <BulletDynamics/ConstraintSolver/btSolverConstraint.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <LinearMath/btScalar.h>

#include "hdtBulletHelper.h"
#include "hdtConstraintGroup.h"

namespace hdt
{
    class SolverBodyMt
    {
    public:
        SolverBodyMt() = default;
        ~SolverBodyMt() = default;

        btSolverBody* m_body;

        auto lock() -> void { m_lock.lock(); }
        auto unlock() -> void { m_lock.unlock(); }

        SolverBodyMt(const SolverBodyMt& rhs) :
            m_body(rhs.m_body) {}

        auto operator=(const SolverBodyMt& rhs) -> SolverBodyMt&
        {
            m_body = rhs.m_body;
            return *this;
        }

    private:
        SpinLock m_lock;
    };

    class SolverTask
    {
    public:
        SolverTask(SolverBodyMt* A, SolverBodyMt* B);

        virtual ~SolverTask() = default;

        virtual auto solve() -> void = 0;

    protected:
        SolverBodyMt* m_bodyA;
        SolverBodyMt* m_bodyB;
        SolverBodyMt* m_lockOrderA;
        SolverBodyMt* m_lockOrderB;
    };

    using SolverTaskPtr = std::shared_ptr<SolverTask>;

    class NonContactSolverTask : public SolverTask
    {
    public:
        NonContactSolverTask(SolverBodyMt* A, SolverBodyMt* B, btSolverConstraint** begin, btSolverConstraint** end,
                             btSingleConstraintRowSolver s);
        auto solve() -> void override;

    protected:
        btSolverConstraint** m_begin;
        btSolverConstraint** m_end;
        btSingleConstraintRowSolver m_solver;
    };

    class ObsoleteSolverTask : public SolverTask
    {
    public:
        ObsoleteSolverTask(SolverBodyMt* A, SolverBodyMt* B, btTypedConstraint* c, float t);
        auto solve() -> void override;

    protected:
        float m_timeStep;
        btTypedConstraint* m_constraint;
    };

    class ContactSolverTask : public SolverTask
    {
    public:
        ContactSolverTask(SolverBodyMt* A, SolverBodyMt* B, btSolverConstraint* c, btSolverConstraint* f0,
                          btSolverConstraint* f1, btSingleConstraintRowSolver sl, btSingleConstraintRowSolver s);
        auto solve() -> void override;

    protected:
        btSolverConstraint* m_contact;
        btSolverConstraint* m_friction0;
        btSolverConstraint* m_friction1;

        btSingleConstraintRowSolver m_solver;
        btSingleConstraintRowSolver m_solverLowerLimit;
    };

    class GroupConstraintSolver : public btSequentialImpulseConstraintSolverMt
    {
        using Base = btSequentialImpulseConstraintSolverMt;

    public:
        GroupConstraintSolver();

        auto solveSingleIteration(int iteration, btCollisionObject** bodies, int numBodies,
                                  btPersistentManifold** manifoldPtr, int numManifolds,
                                  btTypedConstraint** constraints, int numConstraints,
                                  const btContactSolverInfo& infoGlobal,
                                  btIDebugDraw* debugDrawer) -> btScalar override;
        auto solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies,
                                          btPersistentManifold** manifoldPtr, int numManifolds,
                                          btTypedConstraint** constraints, int numConstraints,
                                          const btContactSolverInfo& infoGlobal,
                                          btIDebugDraw* debugDrawer) -> btScalar override;
        auto solveGroupCacheFriendlyFinish(btCollisionObject** bodies, int numBodies,
                                           const btContactSolverInfo& infoGlobal) -> btScalar override;

        static auto getResolveSingleConstraintRowGenericAVX() -> btSingleConstraintRowSolver;
        static auto getResolveSingleConstraintRowLowerLimitAVX() -> btSingleConstraintRowSolver;

        std::vector<ConstraintGroup*> m_groups;
        std::vector<SolverBodyMt> m_bodiesMt;
        std::vector<btSolverConstraint*> m_nonContactConstraintRowPtrs;
        std::vector<SolverTaskPtr> m_tasks;
        std::vector<SolverTaskPtr> m_contactTasks;
        std::vector<SolverTaskPtr> m_nonContactTasks;
    };
} // namespace hdt
