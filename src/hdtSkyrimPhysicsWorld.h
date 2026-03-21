#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>

#include <BulletDynamics/ConstraintSolver/btContactSolverInfo.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <LinearMath/btVector3.h>
#include <RE/B/BSTEvent.h>
#include <RE/N/NiPoint3.h>
#include <SKSE/Events.h>
#include <ppl.h>

#include "Events.h"
#include "hdtConvertNi.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"

namespace hdt
{
    inline constexpr float RESET_PHYSICS = -10.0f;

    class SkyrimPhysicsWorld :
        protected SkinnedMeshWorld,
        public RE::BSTEventSink<Events::FrameEvent>,
        public RE::BSTEventSink<Events::ShutdownEvent>,
        public RE::BSTEventSink<SKSE::CameraEvent>,
        public RE::BSTEventSink<Events::FrameSyncEvent>
    {
    public:
        static auto get() -> SkyrimPhysicsWorld*;

        auto doUpdate(float delta) -> void;
        auto doUpdate2ndStep(float delta, const float tick, const float remainingTimeStep) -> void;
        auto updateActiveState() const -> void;

        auto addSkinnedMeshSystem(SkinnedMeshSystem* system) -> void override;
        auto removeSkinnedMeshSystem(SkinnedMeshSystem* system) -> void override;
        auto removeSystemByNode(void* root) -> void;

        auto resetTransformsToOriginal() -> void override;
        auto resetSystems() -> void;

        auto ProcessEvent(const Events::FrameEvent* e,
                          RE::BSTEventSource<Events::FrameEvent>*) -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const Events::FrameSyncEvent* e,
                          RE::BSTEventSource<Events::FrameSyncEvent>*) -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const Events::ShutdownEvent* e,
                          RE::BSTEventSource<Events::ShutdownEvent>*) -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const SKSE::CameraEvent* evn,
                          RE::BSTEventSource<SKSE::CameraEvent>* dispatcher) -> RE::BSEventNotifyControl override;

        auto isSuspended() const -> bool { return m_suspended; }

        auto suspend(bool loading = false) -> void
        {
            m_suspended = true;
            m_loading = loading;
        }

        auto resume() -> void
        {
            m_suspended = false;
            if (m_loading)
            {
                resetSystems();
                m_loading = false;
            }
        }

        auto suspendSimulationUntilFinished(const std::function<void(void)>& process) -> void;
        std::atomic_bool m_isStasis = false;

        auto applyTranslationOffset() -> btVector3;
        auto restoreTranslationOffset(const btVector3&) -> void;

        auto getSolverInfo() -> btContactSolverInfo& { return btDiscreteDynamicsWorld::getSolverInfo(); }

        // @brief setWind force value for the world
        // @param a_direction wind direction
        // @a_scale Amount to scale the windForce. Defaults to scaleSkyrim
        // @a_smoothingSamples How many samples to smooth. Defaults to 8. Must be greater than 0. Value of 1 means no smoothing
        auto setWind(const RE::NiPoint3& a_direction, float a_scale = scaleSkyrim,
                     uint32_t a_smoothingSamples = 8) -> void;

        concurrency::task_group m_tasks;

        bool m_useRealTime = false;
        int min_fps = 60;
        int m_percentageOfFrameTime = 300;
        // percentage of time per frame doing hdt. Profiler shows 30% is reasonable. Out of 1000.
        float m_timeTick = 1 / 60.f;
        int m_maxSubSteps = 4;
        bool m_clampRotations = true;
        // @brief rotation speed limit of the PC in radians per second. Must be positive.
        float m_rotationSpeedLimit = 10.f;
        bool m_unclampedResets = true;
        float m_unclampedResetAngle = 120.0f;
        float m_2ndStepAverageProcessingTime = 0;
        float m_averageSMPProcessingTimeInMainLoop = 0;
        bool disabled = false;
        uint8_t m_resetPc;
        bool m_doMetrics = false;
        int m_sampleSize = 5;
        // how many samples (each sample taken every second) for determining average time per activeSkeleton.

        //wind settings
        bool m_enableWind = true;
        float m_windStrength = 2.0f; // compare to gravity acceleration of 9.8
        float m_distanceForNoWind = 50.0f; // how close to wind obstruction to fully block wind
        float m_distanceForMaxWind = 3000.0f; // how far to wind obstruction to not block wind

    private:
        SkyrimPhysicsWorld();
        ~SkyrimPhysicsWorld() override = default;

        std::mutex m_lock;

        std::atomic_bool m_suspended;
        std::atomic_bool m_loading;
        float m_accumulatedInterval;
        float m_averageInterval;
        float m_SMPProcessingTimeInMainLoop = 0;
    };
}
