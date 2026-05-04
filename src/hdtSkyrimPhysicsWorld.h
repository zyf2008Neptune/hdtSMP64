#pragma once

#include "ActorManager.h"
#include "Events.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
    constexpr float RESET_PHYSICS = -10.0f;

    class SkyrimPhysicsWorld : protected SkinnedMeshWorld,
                               public RE::BSTEventSink<Events::FrameEvent>,
                               public RE::BSTEventSink<Events::ShutdownEvent>,
                               public RE::BSTEventSink<SKSE::CameraEvent>,
                               public RE::BSTEventSink<Events::FrameSyncEvent>
    {
    public:
        static auto get() -> SkyrimPhysicsWorld*;

        auto doUpdate(float delta) -> void;
        auto doUpdate2ndStep(float delta, float tick, float remainingTimeStep) -> void;
        auto updateActiveState() const -> void;
        auto setProfilerCapture(bool a_enabled, std::uint64_t a_sampleFrames = 240, std::uint64_t a_printFrames = 240)
            -> void;

        auto addSkinnedMeshSystem(SkinnedMeshSystem* system) -> void override;
        auto removeSkinnedMeshSystem(SkinnedMeshSystem* system) -> void override;
        auto removeSystemByNode(void* root) -> void;
        using SkinnedMeshWorld::updateConstraintsForBone;
        auto resetTransformsToOriginal() -> void;
        auto resetSystems() -> void;

        auto ProcessEvent(const Events::FrameEvent* e, RE::BSTEventSource<Events::FrameEvent>*)
            -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const Events::FrameSyncEvent* e, RE::BSTEventSource<Events::FrameSyncEvent>*)
            -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const Events::ShutdownEvent* e, RE::BSTEventSource<Events::ShutdownEvent>*)
            -> RE::BSEventNotifyControl override;
        auto ProcessEvent(const SKSE::CameraEvent* evn, RE::BSTEventSource<SKSE::CameraEvent>* dispatcher)
            -> RE::BSEventNotifyControl override;

        auto isSuspended() -> bool { return m_suspended; }

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

        // This is used for when you want to mutate some physics objects without causing problems
        // Bullet is VERY sensitive to changes during simulation!
        auto lockSimulation() -> std::unique_lock<std::mutex>;

        auto applyTranslationOffset() -> btVector3;
        auto restoreTranslationOffset(const btVector3&) -> void;

        auto getSolverInfo() -> btContactSolverInfo& { return btDiscreteDynamicsWorld::getSolverInfo(); }

        // @brief setWind force value for the world
        // @param a_direction wind direction
        // @a_scale Amount to scale the windForce. Defaults to scaleSkyrim
        // @a_smoothingSamples How many samples to smooth. Defaults to 8. Must be greater than 0. Value of 1 means no
        // smoothing
        auto setWind(const RE::NiPoint3& a_direction, float a_scale = scaleSkyrim, uint32_t a_smoothingSamples = 8)
            -> void;

        tbb::task_group m_tasks;

        bool m_pendingTransformUpdate = false;
        bool m_useRealTime = false;
        int min_fps = 60;
        float m_budgetMs = 3.5f;
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

        // wind settings
        bool m_enableWind = true;
        float m_windStrength = 2.0f; // compare to gravity acceleration of 9.8
        float m_distanceForNoWind = 50.0f; // how close to wind obstruction to fully block wind
        float m_distanceForMaxWind = 3000.0f; // how far to wind obstruction to not block wind

    private:
        SkyrimPhysicsWorld();
        ~SkyrimPhysicsWorld() noexcept override = default;

        std::mutex m_lock;

        std::atomic_bool m_suspended;
        std::atomic_bool m_loading;
        float m_accumulatedInterval;
        float m_averageInterval;
        float m_SMPProcessingTimeInMainLoop = 0;
    };
} // namespace hdt
