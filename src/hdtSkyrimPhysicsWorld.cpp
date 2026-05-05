#include "hdtSkyrimPhysicsWorld.h"

#include <LinearMath/btQuickprof.h>

#include "PluginInterfaceImpl.h"
#include "WeatherManager.h"
#include "hdtPhysicsProfiler.h"

namespace hdt
{
    namespace
    {
        [[maybe_unused]] const float* timeStamp = reinterpret_cast<float*>(0x12E355C);
    }

    SkyrimPhysicsWorld::SkyrimPhysicsWorld()
    {
        gDisableDeactivation = true;
        btDiscreteDynamicsWorld::setGravity(btVector3(0, 0, -9.8f * scaleSkyrim));

        // https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h

        getSolverInfo().m_friction = 0;

        // This should be enabled by default, but just for clarity I put it here too
        getSolverInfo().m_splitImpulse = true;

        // Set a very low threshold so even micro-penetrations use Split Impulse
        // Too low might cause weird visuals - default is -0.04f
        getSolverInfo().m_splitImpulsePenetrationThreshold = -0.01f;

        // Default ERP2 is 0.2
        // From Bullet: error reduction for non-contact constraints
        getSolverInfo().m_erp2 = 0.15f;

        // constraint force mixing for contacts and non-contacts
        // Adds "sponginess" to collisions to absorb the constant recalculations
        // Default is 0
        getSolverInfo().m_globalCfm = 0.001f;

        // Ignore Bounciness (Restitution) on slow micro-collisions
        // If objects are moving slower than this, they will not bounce at all.
        // The default is 0.2f, but putting this here since it's very noteworthy!
        getSolverInfo().m_restitutionVelocityThreshold = 0.2f;

        // Default is = SOLVER_USE_WARMSTARTING | SOLVER_SIMD;
        // But we don't even use warm starts since we delete the manifolds every frame
        // SOLVER_SIMD nets a small performance uplift
        // SOLVER_RANDMIZE_ORDER is also possible, but I clocked a pretty heavy performance hit. Maybe make it a config
        // option
        getSolverInfo().m_solverMode = SOLVER_SIMD;
        getSolverInfo().m_leastSquaresResidualThreshold = 0.0001f;

        m_averageInterval = m_timeTick;
        m_accumulatedInterval = 0;
    }

    auto SkyrimPhysicsWorld::setProfilerCapture(const bool a_enabled, const std::uint64_t a_sampleFrames,
                                                const std::uint64_t a_printFrames) -> void
    {
        auto simulationLock = lockSimulation();
        physicsprofiler::setCapture(a_enabled, a_sampleFrames, a_printFrames);
    }

    // void hdtSkyrimPhysicsWorld::suspend()
    //{
    //	m_suspended++;
    // }

    // void hdtSkyrimPhysicsWorld::resume()
    //{
    //	--m_suspended;
    // }

    // void hdtSkyrimPhysicsWorld::switchToSeperateClock()
    //{
    //	m_lock.lock();
    //	m_useSeperatedClock = true;
    //	m_timeLastUpdate = clock()*0.001;
    //	m_lock.unlock();
    // }

    // void hdtSkyrimPhysicsWorld::switchToInternalClock()
    //{
    //	m_lock.lock();
    //	m_useSeperatedClock = false;
    //	m_timeLastUpdate = *timeStamp;
    //	m_lock.unlock();
    // }

    auto SkyrimPhysicsWorld::get() -> SkyrimPhysicsWorld*
    {
        static SkyrimPhysicsWorld g_World;
        return &g_World;
    }

    auto SkyrimPhysicsWorld::doUpdate(float interval) -> void
    {
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

        // Time passed since last computation
        m_accumulatedInterval += interval;

        // Exponential average - becomes the tick; the tick equals the average interval when the interval is stable.
        m_averageInterval += (interval - m_averageInterval) * .125f;

        // No need to calculate physics if there is no active skeleton.
        if (!disabled && hdt::ActorManager::instance()->activeSkeletons)
        {
            // The tick is the given time for each computation substep. We set it to the average fps
            // to have one average computation each frame when everything is usual.
            // In case of poor fps, we set it to the configured minimum engine value (60 Hz),
            // to still allow a physics with max increments of 1/60s.
            const auto tick = std::min(m_averageInterval, m_timeTick);

            // No need to calculate physics when too little time has passed (time exceptionally short since last
            // computation). This magic value directly impacts the number of computations and the time cost of the
            // mod...
            if (m_accumulatedInterval * 2.0f > tick)
            {
                // The interval is limited to a configurable number of substeps, by default 4.
                // Additional substeps happens when there is a very sudden slowdown, or when fps is lower than min-fps,
                // we have to compute for the passed time we haven't computed.
                // n substeps means that when instant fps is n times lower than usual current fps, we stop computing.
                // So, we guarantee no jitter for fps greater than min-fps / maxSubsteps.
                // For example, if min-fps = 60 and maxSubsteps = 4, we guarantee no jitter for 15+ fps,
                // at the cost of additional simulations.
                const auto remainingTimeStep = std::min(m_accumulatedInterval, tick * m_maxSubSteps);

                readTransform(remainingTimeStep);

                m_resetPc -= m_resetPc > 0;

                m_tasks.run([this, interval, tick, remainingTimeStep]
                            { doUpdate2ndStep(interval, tick, remainingTimeStep); });
            }
        }
    }

    auto SkyrimPhysicsWorld::doUpdate2ndStep(float, const float tick, const float remainingTimeStep) -> void
    {
        if (m_suspended)
        {
            return;
        }

        std::scoped_lock l(m_lock);

        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

        LARGE_INTEGER ticks;
        int64_t startTime = 0;
        if (m_doMetrics)
        {
            QueryPerformanceCounter(&ticks);
            startTime = ticks.QuadPart;
        }

        g_pluginInterface.onPreStep({.objects = getCollisionObjectArray(), .timeStep = remainingTimeStep});

        {
            BT_PROFILE("HDTSMP_doUpdate2ndStep");
            updateActiveState();
            const auto offset = applyTranslationOffset();
            stepSimulation(remainingTimeStep, 0, tick);
            restoreTranslationOffset(offset);
            m_accumulatedInterval = 0;
            m_pendingTransformUpdate = true;
        }

        g_pluginInterface.onPostStep({.objects = getCollisionObjectArray(), .timeStep = remainingTimeStep});

        if (m_doMetrics)
        {
            QueryPerformanceCounter(&ticks);
            const int64_t endTime = ticks.QuadPart;
            QueryPerformanceFrequency(&ticks);
            // float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
            const float lastProcessingTime = (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
            m_2ndStepAverageProcessingTime = (m_2ndStepAverageProcessingTime + lastProcessingTime) * 0.5f;
        }

        physicsprofiler::advanceFrame();
    }

    auto SkyrimPhysicsWorld::lockSimulation() -> std::unique_lock<std::mutex>
    {
        m_tasks.wait();
        return std::unique_lock(m_lock);
    }

    auto SkyrimPhysicsWorld::applyTranslationOffset() -> btVector3
    {
        btVector3 center;
        center.setZero();
        int count = 0;
        for (int i = 0; i < m_collisionObjects.size(); ++i)
        {
            if (const auto rig = btRigidBody::upcast(m_collisionObjects[i]))
            {
                center += rig->getWorldTransform().getOrigin();
                ++count;
            }
        }

        if (count > 0)
        {
            center /= static_cast<btScalar>(count);
            for (int i = 0; i < m_collisionObjects.size(); ++i)
            {
                if (const auto rig = btRigidBody::upcast(m_collisionObjects[i]))
                {
                    rig->getWorldTransform().getOrigin() -= center;
                }
            }
        }
        return center;
    }

    auto SkyrimPhysicsWorld::restoreTranslationOffset(const btVector3& offset) -> void
    {
        for (int i = 0; i < m_collisionObjects.size(); ++i)
        {
            if (const auto rig = btRigidBody::upcast(m_collisionObjects[i]))
            {
                rig->getWorldTransform().getOrigin() += offset;
            }
        }
    }

    auto SkyrimPhysicsWorld::setWind(const RE::NiPoint3& a_point, const float a_scale, uint32_t a_smoothingSamples)
        -> void
    {
        if (a_smoothingSamples == 0)
        {
            logger::error("setWind a_smoothingSamples must be > 0; values ignored");
            return;
        }
        const auto oldValueWeight = a_smoothingSamples - 1;
        if (!btFuzzyZero((m_windSpeed - btVector3(a_point.x, a_point.y, a_point.z)).length()))
        {
            m_windSpeed.setValue((oldValueWeight * m_windSpeed.getX() + a_point.x * a_scale) / a_smoothingSamples,
                                 (oldValueWeight * m_windSpeed.getY() + a_point.y * a_scale) / a_smoothingSamples,
                                 (oldValueWeight * m_windSpeed.getZ() + a_point.z * a_scale) / a_smoothingSamples);
            logger::debug("Wind Speed now ({:.2f}, {:.2f}, {:.2f}), target ({:.2f}, {:.2f}, {:.2f}) using {} samples.",
                          m_windSpeed.getX(), m_windSpeed.getY(), m_windSpeed.getZ(), a_point.x * a_scale,
                          a_point.y * a_scale, a_point.z * a_scale, a_smoothingSamples);
        }
    }

    auto SkyrimPhysicsWorld::updateActiveState() const -> void
    {
        struct Group
        {
            std::unordered_set<RE::BSFixedString> tags;
            std::unordered_map<RE::BSFixedString, std::vector<SkyrimBody*>> list;
        };

        std::unordered_map<RE::NiNode*, Group> maps;

        const RE::BSFixedString invalidString;
        for (auto& i : m_systems)
        {
            const auto system = dynamic_cast<SkyrimSystem*>(i.get());
            auto& map = maps[system->m_skeleton.get()];
            for (auto& j : system->meshes())
            {
                auto shape = static_cast<SkyrimBody*>(j.get()); // use downcast is safety here
                if (!shape)
                {
                    continue;
                }

                if (shape->m_disableTag == invalidString)
                {
                    for (auto& k : shape->m_tags)
                    {
                        map.tags.insert(k);
                    }
                }
                else
                {
                    map.list[shape->m_disableTag].push_back(shape);
                }
            }
        }

        for (auto& val : maps | std::views::values)
        {
            for (auto& [fst, snd] : val.list)
            {
                if (val.tags.contains(fst))
                {
                    for (const auto& k : snd)
                    {
                        k->m_disabled = true;
                    }
                }
                else if (!snd.empty())
                {
                    std::ranges::sort(snd,
                                      [](const SkyrimBody* a, const SkyrimBody* b)
                                      {
                                          if (a->m_disablePriority != b->m_disablePriority)
                                          {
                                              return a->m_disablePriority > b->m_disablePriority;
                                          }
                                          return a < b;
                                      });

                    for (const auto& k : snd)
                    {
                        k->m_disabled = true;
                    }
                    snd[0]->m_disabled = false;
                }
            }
        }
    }

    auto SkyrimPhysicsWorld::addSkinnedMeshSystem(SkinnedMeshSystem* system) -> void
    {
        std::scoped_lock l(m_lock);
        const auto s = static_cast<SkyrimSystem*>(system); // use downcast is safety here
        if (!s)
        {
            return;
        }

        s->m_initialized = false;
        SkinnedMeshWorld::addSkinnedMeshSystem(system);
    }

    auto SkyrimPhysicsWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system) -> void
    {
        std::scoped_lock l(m_lock);

        SkinnedMeshWorld::removeSkinnedMeshSystem(system);
    }

    auto SkyrimPhysicsWorld::removeSystemByNode(void* root) -> void
    {
        std::scoped_lock l(m_lock);

        for (int i = 0; i < m_systems.size();)
        {
            RE::BSTSmartPointer<SkyrimSystem> s =
                hdt::make_smart(static_cast<SkyrimSystem*>(m_systems[i].get())); // use downcast is safety here
            if (s && s->m_skeleton == root)
            {
                SkinnedMeshWorld::removeSkinnedMeshSystem(s.get());
            }

            else
            {
                ++i;
            }
        }
    }

    auto SkyrimPhysicsWorld::resetSystems() -> void
    {
        std::scoped_lock l(m_lock);
        for (const auto& i : m_systems)
        {
            i->readTransform(i->prepareForRead(RESET_PHYSICS));
        }
    }

    auto SkyrimPhysicsWorld::ProcessEvent(const Events::FrameEvent* e, RE::BSTEventSource<Events::FrameEvent>*)
        -> RE::BSEventNotifyControl
    {
        const auto mm = RE::UI::GetSingleton();

        if ((e->gamePaused || mm->GameIsPaused()) && !m_suspended)
        {
            suspend();
        }
        else if (!(e->gamePaused || mm->GameIsPaused()) && m_suspended)
        {
            resume();
        }

        if (m_enableWind)
        {
            WeatherManager::runWeatherTick(RE::GetSecondsSinceLastFrame());
        }

        LARGE_INTEGER ticks;
        int64_t startTime = 0;
        int64_t endTime = 0;
        if (m_doMetrics)
        {
            QueryPerformanceCounter(&ticks);
            startTime = ticks.QuadPart;
        }

        std::scoped_lock l(m_lock);

        const float interval =
            m_useRealTime ? RE::BSTimer::GetSingleton()->realTimeDelta : RE::BSTimer::GetSingleton()->delta;

        if (interval > FLT_EPSILON && !m_suspended && !m_systems.empty())
        {
            doUpdate(interval);
        }
        else if (m_suspended && !m_loading)
        {
            writeTransform();
        }

        if (m_doMetrics)
        {
            QueryPerformanceCounter(&ticks);
            endTime = ticks.QuadPart;
            QueryPerformanceFrequency(&ticks);
            // float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
            m_SMPProcessingTimeInMainLoop = (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
        }

        return RE::BSEventNotifyControl::kContinue;
    }

    auto SkyrimPhysicsWorld::ProcessEvent(const Events::FrameSyncEvent*, RE::BSTEventSource<Events::FrameSyncEvent>*)
        -> RE::BSEventNotifyControl
    {
        if (m_doMetrics)
        {
            LARGE_INTEGER ticks, freq;
            QueryPerformanceCounter(&ticks);
            const int64_t t0 = ticks.QuadPart;

            m_tasks.wait();

            QueryPerformanceCounter(&ticks);
            const int64_t t1 = ticks.QuadPart;
            if (m_pendingTransformUpdate)
            {
                std::scoped_lock l(m_lock);
                writeTransform();
                m_pendingTransformUpdate = false;
            }

            QueryPerformanceCounter(&ticks);
            const int64_t t2 = ticks.QuadPart;
            QueryPerformanceFrequency(&freq);
            const float f = static_cast<float>(freq.QuadPart);

            const float instWaitTime = (t1 - t0) / f * 1000.0f;
            const float instWriteTime = (t2 - t1) / f * 1000.0f;
            const float instSetupTime = m_SMPProcessingTimeInMainLoop;

            const float instFpsImpact = instSetupTime + instWaitTime + instWriteTime;

            m_averageSMPProcessingTimeInMainLoop =
                (m_averageSMPProcessingTimeInMainLoop * (m_sampleSize - 1) + instFpsImpact) / m_sampleSize;

            // Smooth the individual components for logging so the math adds up perfectly visually
            static float avgSetupTime = 0.0f;
            static float avgWaitTime = 0.0f;
            static float avgWriteTime = 0.0f;

            avgSetupTime = (avgSetupTime * (m_sampleSize - 1) + instSetupTime) / m_sampleSize;
            avgWaitTime = (avgWaitTime * (m_sampleSize - 1) + instWaitTime) / m_sampleSize;
            avgWriteTime = (avgWriteTime * (m_sampleSize - 1) + instWriteTime) / m_sampleSize;

            // The background thread's math time (this is already smoothed in doUpdate2ndStep)
            const float avgBackgroundCalcTime = m_2ndStepAverageProcessingTime;

            // How much of that background math was successfully hidden?
            float avgHiddenTime = std::max(0.0f, avgBackgroundCalcTime - avgWaitTime);

            float avgTotalCpuWork = avgSetupTime + avgBackgroundCalcTime + avgWriteTime;

            logger::info(
                "[SMP Metrics] Avg Frame-time Impact: {:.2f}ms (Setup: {:.2f}, Wait: {:.2f}, Apply: {:.2f}) | Avg "
                "Hidden Time: {:.2f}ms | Avg Total CPU Work: {:.2f}ms",
                m_averageSMPProcessingTimeInMainLoop, // This will exactly equal Setup + Wait + Apply
                avgSetupTime, avgWaitTime, avgWriteTime, avgHiddenTime, avgTotalCpuWork);
        }
        else
        {
            m_tasks.wait();
            if (m_pendingTransformUpdate)
            {
                std::scoped_lock l(m_lock);
                writeTransform();
                m_pendingTransformUpdate = false;
            }
        }

        return RE::BSEventNotifyControl::kContinue;
    }

    auto SkyrimPhysicsWorld::ProcessEvent(const Events::ShutdownEvent*, RE::BSTEventSource<Events::ShutdownEvent>*)
        -> RE::BSEventNotifyControl
    {
        // m_tasks.wait();
        // std::scoped_lock l(m_lock);

        while (!m_systems.empty())
        {
            SkinnedMeshWorld::removeSkinnedMeshSystem(m_systems.back().get());
        }

        m_tasks.wait();

        return RE::BSEventNotifyControl::kContinue;
    }

    auto SkyrimPhysicsWorld::ProcessEvent(const SKSE::CameraEvent* evn, RE::BSTEventSource<SKSE::CameraEvent>*)
        -> RE::BSEventNotifyControl
    {
        if (evn && evn->oldState && evn->newState)
        {
            if (evn->oldState->id == RE::CameraState::kFirstPerson &&
                evn->newState->id == RE::CameraState::kThirdPerson)
            {
                m_resetPc = 3;
            }
        }

        return RE::BSEventNotifyControl::kContinue;
    }
} // namespace hdt
